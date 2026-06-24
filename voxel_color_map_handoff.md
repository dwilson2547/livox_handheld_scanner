# Handoff: Probabilistic Voxel Map with Per-Voxel Color Accumulation & Outlier Rejection

## Purpose

Replace naive "project color onto mesh, last-write-wins" coloring with a probabilistic
voxel map that (a) classifies real surfaces vs. noise via occupancy confidence and
(b) accumulates per-voxel color robustly so rolling-shutter fliers, reflections, and
mis-projected frames get rejected instead of muddying the result.

This is grounded in two established patterns: **OctoMap-style log-odds occupancy** for
the geometry confidence, and **robust (median / weighted) color accumulation** for the
color. We are not inventing from scratch; we are combining two known-good ideas.

## Prerequisite (do this FIRST — see camera_lidar_calibration_handoff.md)

Do **not** build this until a single static-frame projection test confirms the
camera→LiDAR extrinsic is correct. A robust color accumulator still places color in the
**wrong location** if the extrinsic rotation is wrong — it will just place wrong-location
color *confidently*. Verify calibration before investing here.

---

## System Overview

The world is pre-divided into voxels. Each voxel independently tracks:

1. **Occupancy** — a log-odds probability that a real surface exists in this voxel.
2. **Color** — a robust estimate built from many weighted samples over the scan.

On each sensor frame, for every LiDAR point we update the occupancy of the voxel it
lands in (and clear voxels the ray passed through), and — if the voxel is visible to the
camera and not occluded — add one weighted color sample.

At export, we emit only high-occupancy voxels, colored by their robust color estimate.

---

## Voxel Size — Critical Decision

**Do not default to 1cm.** Handheld LIO trajectory accuracy is ~1–2cm. If voxels are
smaller than pose error, the *same physical surface* registers into *different* voxels on
different passes, so the "paint the same voxel 50 times" assumption partially breaks and
hits scatter across a neighborhood.

Options:
- **2cm voxels (recommended start):** repeated passes reliably land in the same voxel.
  Color accumulation works as intended.
- **1cm voxels:** only if you accumulate color over a small neighborhood (voxel + 26
  neighbors) to absorb pose jitter. More complex; defer.

Make voxel size a config parameter. Start at 0.02m, validate, then experiment downward.

---

## Data Structures

Use a **spatial hash map** keyed by integer voxel coordinates (sparse — most of the world
is empty, never allocate a dense grid).

```
VoxelKey   = (int x, int y, int z)            # world_coord / voxel_size, floored
VoxelKey hashing: combine the three ints (e.g. boost hash_combine or a 64-bit Morton code)

Voxel {
    float   log_odds          = 0.0           # occupancy; 0 = unknown (p=0.5)
    uint16  hit_count         = 0             # diagnostics / min-sample gating
    ColorAccumulator color                    # see below
}

VoxelMap = hash_map<VoxelKey, Voxel>
```

### ColorAccumulator (the part that actually fixes blotchiness)

**Do NOT use a running mean.** A naive average is permanently dragged by one bad frame and
is the most likely current cause of muddy color. Use one of:

**Option A — Weighted running median (recommended).**
Store a bounded reservoir of recent samples (e.g. last 64) as `(rgb, weight)`, output the
weighted median per channel at export. Bounded memory, robust to fliers.

**Option B — Weighted incremental with robust gate.**
Maintain a weighted mean+variance; reject any incoming sample whose distance from the
current estimate exceeds k·sigma (e.g. k=2.5) before folding it in. Cheaper memory,
slightly less robust on early samples (seed with first few unconditionally).

Start with Option A for correctness; switch to B only if memory becomes a problem at scale.

```
ColorAccumulator {
    # Option A
    ring_buffer<(rgb8, float weight)> samples   # cap N (e.g. 64)
    add(rgb, weight): push, evict oldest if full
    result(): per-channel weighted median
}
```

---

## Per-Sample Color Weight (compute at capture time — this is the rolling-shutter fix)

Each color sample gets a confidence weight. Attack rolling shutter and grazing-angle error
at the source rather than catching it downstream. Multiply these factors:

| Factor | Rationale | Weight shape |
|---|---|---|
| **View angle** | Face-on surfaces project cleanly; grazing angles smear & alias | `cos(theta)` between voxel surface normal and camera ray; clamp low, drop below ~70° |
| **Range** | D435 RGB↔depth alignment degrades with distance | falls off with range; e.g. `1/(1+range²)` or a soft cap beyond ~5m |
| **Angular velocity** | High rotation rate → rolling-shutter skew. **You already have ω from the IMU/trajectory — use it.** | `exp(-k·|ω|)`; down-weight fast-motion frames |
| **(optional) Exposure/blur** | Down-weight blurry/over-dark frames | from image sharpness metric or exposure metadata |

The angular-velocity weight is the highest-leverage one and is nearly free because the
trajectory already carries ω. A single bad rolling-shutter frame during a fast pan gets a
low weight and is dominated by the 50+ clean frames of the same voxel.

---

## Per-Frame Update Loop

```
for each frame f (LiDAR sweep + synced RGB image):
    pose      = trajectory.interpolate(f.lidar_timestamp)   # pose at THIS frame's time
    cam_pose  = pose ∘ T_lidar_cam                          # apply extrinsic
    ω         = trajectory.angular_velocity(f.lidar_timestamp)

    # --- occupancy: ray clearing + endpoint hit ---
    for each point p in f.points:
        # clear voxels along the ray from sensor origin to p (miss evidence)
        for each voxel v on ray(origin → p) excluding endpoint:
            v.log_odds += L_MISS         # decrement (negative); clamp to L_MIN
        # endpoint hit
        v_end = voxel_of(p)
        v_end.log_odds += L_HIT          # increment; clamp to L_MAX
        v_end.hit_count++

    # --- color: project visible, non-occluded voxels ---
    img_w = compute_motion_weight(ω)     # frame-level part of the weight
    for each voxel v with v.log_odds > L_OCC_MIN  in camera frustum:
        uv, depth_v = project(v.center, cam_intrinsics, cam_pose)   # pinhole + distortion
        if uv outside image: continue
        if occluded(uv, depth_v, f.depth_image): continue           # z-test vs D435 depth
        sample_w = img_w * view_angle_weight(v, cam_pose) * range_weight(depth_v)
        v.color.add( rgb_at(uv, f.rgb_image), sample_w )
```

### Log-odds constants (OctoMap-style starting values)
```
L_HIT   = +0.85     # ln(0.7/0.3)
L_MISS  = -0.40     # ln(0.4/0.6)
L_MIN   = -2.0      # clamp (prevents over-confidence, allows recovery)
L_MAX   = +3.5      # clamp
L_OCC_MIN = +0.85   # threshold to consider a voxel "occupied" for coloring/export
```
Tune `L_OCC_MIN` per scene — this is your **noise threshold knob** (the tunable you asked
for). Raise it to reject more aggressively in noisy/reflective scenes.

### Why ray-clearing matters (the noise-rejection core)
A spurious reflection hits a voxel once (one +L_HIT). But subsequent rays to real surfaces
**pass through** that empty space, each applying L_MISS, actively driving the false voxel
back below threshold. Real surfaces get repeated hits and are never passed-through, so they
climb and stay. This is strictly better than a hit-counter because misses are evidence too.

---

## Occlusion Test

A voxel can be inside the camera frustum but behind a nearer surface. Without a test, hidden
voxels grab foreground color (a classic blotch source).

```
occluded(uv, depth_voxel, depth_image):
    d_cam = depth_image.sample(uv)        # D435 gives metric depth directly
    return d_cam is valid AND d_cam < depth_voxel - epsilon   # something closer along ray
```
The robust color accumulator partially absorbs occlusion errors (wrong color is inconsistent
across angles → median rejects), but explicit testing prevents feeding the bad sample at all.
Do both.

---

## Export

```
for each voxel v in map:
    if v.log_odds < L_OCC_MIN: skip                  # noise floor
    if v.color.sample_count < N_MIN_COLOR: flag/fill # low-confidence color (e.g. 3)
    emit voxel at v.center, color = v.color.result()
```
Feed surviving voxels to your existing meshing (VDBFusion / marching cubes). Note: VDBFusion
is itself a TSDF voxel integrator, so confirm what it already does before duplicating —
you may only need to add the **color robustness layer** on top of its existing geometry
confidence rather than reimplementing occupancy.

---

## Build / Validation Order

1. **Confirm calibration first** (other handoff). Non-negotiable.
2. Build voxel hash map + log-odds occupancy. Validate geometry-only: does noise drop out,
   do real surfaces persist? Visualize occupancy before touching color.
3. Add color accumulation with **uniform weights + median**. Confirm color lands in the
   right place and fliers are rejected.
4. Add the **weighted** sample confidence (view angle, range, angular velocity). Confirm
   rolling-shutter frames stop contaminating.
5. Add occlusion test. Confirm foreground color stops bleeding onto background.
6. Tune `L_OCC_MIN` and voxel size per scene.

## Known Traps
- Naive mean color → muddy. Use median/robust. (Most likely current bug.)
- Voxel smaller than pose error → hits scatter. Start 2cm.
- Projecting color at the wrong pose (nearest LiDAR frame instead of image's own
  timestamp) → motion smear. Interpolate trajectory to each image's exact time.
- Skipping occlusion → foreground color on background.
- Over-confident log-odds with no clamp → map can't recover from a transient error. Clamp.