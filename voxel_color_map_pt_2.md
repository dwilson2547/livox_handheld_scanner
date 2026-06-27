# Handoff Part 2: Refinements to the Probabilistic Voxel Color Map

**Companion to `voxel_color_map_handoff.md` — does not replace it.**
The original spec is sound and buildable as written. This doc captures refinements
based on what we've learned since it was authored (IMU role, extrinsic validation,
storage constraints) plus a few corrections that prevent building the wrong thing.
Read the original first; this assumes its data structures and update loop.

---

## 0. Status check that motivates this doc

- **Extrinsic is validated and close.** A static-frame depth-projection overlay (matching
  camera + LiDAR frames, points colored by depth, checked against the image) lined up well.
  Estimated residual is a few mm front-to-back (~2cm worst case), which is negligible at
  room range. **The original's "confirm calibration first" gate is satisfied** — proceed to
  the color layer. The blotchiness is algorithmic, not geometric, exactly as the original
  predicted for the "wrong-location color placed confidently" failure mode being *ruled out*.

- **The IMU is effectively unused in this rig.** We did a deep dive: with Horizon-class point
  density in structured indoor scenes, LiDAR residuals dominate the state update and the IMU
  is along for the ride. This changes where one of the original's weights sources its signal
  (see §1).

---

## 1. CORRECTION: source ω from the trajectory, not the IMU

The original says the angular-velocity weight `exp(-k·|ω|)` is "nearly free because the
trajectory already carries ω... You already have ω from the IMU/trajectory — use it."

That conclusion is still correct, but the wording invites a future reader to go fishing for
an IMU angular-velocity signal we've since decided to ignore. Make it explicit:

> **Compute ω by differentiating the Point-LIO trajectory pose stream, NOT from raw IMU.**
> Because LiDAR dominates the state estimate in this rig, the trajectory ω is *more*
> trustworthy than raw IMU ω, not less. Finite-difference the interpolated rotation
> (`ω ≈ Log(R(t).inverse() * R(t+dt)) / dt`) around each image timestamp.

The weight itself is unchanged and remains the highest-leverage one for rolling-shutter
rejection. Only its provenance changes.

---

## 2. REFINEMENT: lowest-weight eviction, not oldest

The original's ColorAccumulator Option A uses a bounded ring buffer (last 64 samples) and
evicts **oldest** when full. On a thorough scan a voxel can be observed far more than 64
times, so eviction policy decides which observations survive to the median.

**Problem:** oldest-first eviction can discard a clean, face-on, slow-motion early frame in
favor of a recent grazing-angle or fast-pan frame, degrading the surviving set over time.

**Change:** make the buffer a **bounded priority structure keyed on sample weight**. When
full, evict the **lowest-weight** sample (only if the incoming sample outranks it). This
keeps the best-N observations of each voxel regardless of capture order.

```
ColorAccumulator (revised):
    fixed-capacity min-heap of (weight, rgb8), cap N (e.g. 64)
    add(rgb, w):
        if size < N:        push (w, rgb)
        elif w > heap.min_weight:
                            pop min, push (w, rgb)
        else:               drop (incoming is worse than everything we kept)
    result(): per-channel weighted median over retained samples
```

Cost is trivial (N is tiny). This is the single best cheap upgrade to the accumulator.

---

## 3. REFINEMENT: per-channel median can invent colors

Per-channel (independent R, G, B) median can output a triplet that no real sample had — on
a surface with two distinct colors split across observations, you can land on a hue that
existed in neither. Rarely visible, but shows up as odd tints on mixed-color or
high-contrast-edge voxels.

**If you see it:** switch from per-channel median to a **vector median** — return the actual
retained sample minimizing the sum of distances to all other retained samples (Tukey-style
medoid). It's guaranteed to be a color that was really observed. With N=64 it's a cheap
O(N²) over a tiny set, per voxel, at export only.

**Defer** unless artifacts appear. Per-channel is fine for a first build.

---

## 4. RESOLVE BEFORE BUILDING: do not run two voxel grids

The original flags this but leaves it open. Resolve it **before** writing the occupancy half,
because it can delete a large chunk of the work.

VDBFusion is itself a TSDF voxel integrator — it already owns geometry confidence and surface
extraction. If you build a parallel OctoMap-style log-odds occupancy grid *and* keep
VDBFusion, you have two grids doing the same job on the same data.

**Recommended architecture:** let **VDBFusion own geometry**; add **only the robust color
layer** on top of its voxels.

- Skip the log-odds occupancy map, ray-clearing, and `L_*` constants entirely (original §
  update-loop occupancy block + log-odds constants).
- Keep: the projection, occlusion test, per-sample weighting, and the robust color
  accumulator — keyed to VDBFusion's voxel/grid coordinates.
- You lose the original's miss-based noise rejection. Evaluate whether VDBFusion's own TSDF
  weighting already suppresses the spurious-reflection voxels well enough in practice. If it
  does, you're done far cheaper. If specific noise survives, add a thin log-odds layer for
  *just those* rather than reimplementing the whole thing.

**Action:** before any coding, dump what VDBFusion already exposes per-voxel (weight, TSDF
value, whether you can attach arbitrary per-voxel payload like a ColorAccumulator). That one
investigation determines whether you build ~40% or ~90% of the original doc.

---

## 5. NEW PREREQUISITE: lock RGB exposure / white balance at capture

The original's weights decide *which* sample wins per voxel — they reject **outliers**. They
do **not** correct **systematic drift**. If the D435 RGB auto-exposure or auto-WB drifts over
a scan, *every* sample of a given voxel is consistently-but-wrongly colored, the median
faithfully preserves the wrong color, and you get a scan whose color shifts wall-to-wall while
each voxel individually looks "confident."

**Fix is capture-side, not algorithmic:** lock exposure and white balance on the D435 RGB
stream before recording (fixed exposure + fixed WB, or at minimum lock WB and accept fixed
exposure tuned to the room). Add this to the capture checklist alongside the original's
calibration gate. No accumulator can recover color information that was never captured
consistently.

---

## 6. Storage-constrained operating notes (current hardware reality)

Context: ~8–10 GB per 3-minute bag, single 1 TB drive in the OptiPlex. Multi-hour continuous
recording is off the table short-term, and that's fine — **none of the above needs long
sessions.** The robust accumulator's whole value shows up within a single normal-length scan
(dozens of observations per voxel from ordinary handheld coverage). Practical implications:

- **Build and validate against existing short bags.** No new long-form data required for any
  step in the original's build order. Steps 2–6 all validate on a 3-minute room scan.
- **Process offline, delete bags after.** The voxel map and exported colored cloud/mesh are
  tiny relative to the raw bag. Pipeline: record → process to voxel map → export → archive or
  delete the bag. Don't let raw bags accumulate on the 1 TB.
- **If you add storage:** the SATA SSD is the low-friction path (cheap, the port's already
  there, and bag I/O during *recording* is sequential write — SATA SSD is plenty for that).
  A bigger NVMe only matters if you become read-throughput-bound during *processing* of many
  bags, which you aren't yet. Start with a SATA SSD as a scratch/bag landing drive; keep the
  NVMe for OS + working set. Revisit NVMe only if processing I/O becomes the bottleneck.
- **Memory, not disk, is the accumulator's real budget.** A room at 2 cm voxels with a
  bounded N=64 accumulator is well within the OptiPlex's 32 GB. The original's warning about
  unbounded per-voxel sample lists is the thing to avoid — the bounded heap in §2 keeps this
  flat regardless of scan length, which is exactly what you want on constrained hardware.

---

## 7. Revised build order (supersedes original §"Build / Validation Order" only where noted)

1. Calibration gate — **already satisfied** (see §0). Skip re-doing it.
2. **Decide the VDBFusion question (§4) before writing any occupancy code.** This gates how
   much of the original you build.
3. Lock exposure/WB on the RGB stream (§5). Re-record one short validation bag with it locked.
4. If building own occupancy: log-odds map, validate geometry-only as original §2.
   If deferring to VDBFusion: skip to step 5.
5. Color accumulation with **uniform weights + median**, using the **lowest-weight eviction**
   buffer from §2. Confirm color lands correctly and fliers reject.
6. Add weighted samples (view angle, range, **trajectory-derived ω** per §1).
7. Add occlusion test (original is correct as written).
8. Tune `L_OCC_MIN` (if applicable) and voxel size per scene.
9. Only if mixed-color tint artifacts appear: switch to vector median (§3).

---

## Summary of deltas from the original

| # | Type | Change |
|---|---|---|
| 1 | Correction | ω comes from trajectory, not raw IMU (IMU is overruled in this rig) |
| 2 | Refinement | Evict lowest-weight sample, not oldest, from the accumulator buffer |
| 3 | Refinement | Vector median fallback if per-channel median invents colors (defer) |
| 4 | Resolve-first | Don't run two voxel grids; let VDBFusion own geometry, add color only |
| 5 | New prereq | Lock RGB exposure/WB at capture — median can't fix systematic drift |
| 6 | Ops | Process-then-delete bags; SATA SSD as scratch; bounded accumulator keeps RAM flat |

Everything else in the original stands.
