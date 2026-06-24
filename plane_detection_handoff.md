# Handoff: Plane Detection (Ground / Wall / Ceiling) on the Voxel Map

## Purpose

Add structural plane detection to the scanning pipeline: classify surfaces as ground,
wall, ceiling, or other, and optionally extract discrete plane instances (equation +
extent). This plugs into the probabilistic voxel map (see voxel_color_map_handoff.md) and
delivers three payoffs:

1. **Cleaner meshes** — snap inlier points to fitted planes; flat walls instead of bumpy
   noisy ones (big win for 3D printing).
2. **Data reduction** — represent a wall as one plane + extent instead of 50k points.
3. **Degeneracy early-warning** — the count/diversity of visible plane normals is a cheap
   live signal for when SLAM is losing geometric constraints (ties back to the
   sky-pointing degeneracy problem).

## Prerequisites
- Probabilistic voxel map with occupancy confidence built and validated (geometry clean,
  noise rejected). Plane detection runs on **occupied, denoised voxels**, not raw sweeps —
  this is what makes it robust (RANSAC's enemy is outliers; the occupancy threshold already
  removed most).
- **Gravity vector available from the IMU.** This is the key enabler — it turns plane
  classification from a search into a filter. Confirm the trajectory/state exposes a
  gravity-aligned down vector (it should, since the IMU is gravity-observing).

---

## Why gravity makes this easy

Most plane detection is hard because you're searching arbitrary orientations. We're not:
- **Ground / ceiling** = horizontal planes, normals **parallel** to gravity.
- **Walls** = vertical planes, normals **perpendicular** to gravity.

So we pre-classify every surface normal by its angle to gravity, which cleaves the problem
into trivial buckets before any fitting happens.

---

## Stage 1 — Per-Voxel Normals

For each occupied voxel, estimate a surface normal from its occupied neighborhood.

```
for each occupied voxel v:
    neighbors = occupied voxels within radius r (e.g. 2–3 voxels)
    if count(neighbors) < N_MIN (e.g. 6): mark normal invalid, skip
    normal = smallest-eigenvector of covariance(neighbor centers)   # PCA plane normal
    v.normal = orient_consistently(normal)                          # see note
```

Notes:
- Normals from the **accumulated/denoised** voxel map are far cleaner than per-frame raw
  normals — this is a direct benefit of running after the occupancy layer.
- Normal sign is ambiguous from PCA; for gravity classification you only need the **axis**,
  not the sign, so `|dot(normal, gravity)|` suffices and orientation consistency is optional
  at this stage.

---

## Stage 2 — Gravity-Based Classification

```
g = normalized gravity (down) vector
for each voxel v with valid normal:
    a = |dot(v.normal, g)|              # 1.0 = parallel to gravity, 0.0 = perpendicular
    if a > cos(ANGLE_HORIZ):            # e.g. ANGLE_HORIZ = 15° → a > 0.966
        v.class = HORIZONTAL            # floor or ceiling (split by height, Stage 4)
    elif a < cos(90° - ANGLE_VERT):    # e.g. ANGLE_VERT = 15° → a < 0.259
        v.class = VERTICAL             # wall candidate
    else:
        v.class = OTHER                # slopes, furniture, clutter
```

`ANGLE_HORIZ` / `ANGLE_VERT` are tunable tolerances. Real surfaces aren't perfect; 10–20°
is a reasonable start. Tighten for cleaner buildings, loosen for rough/old structures.

This stage alone gives you a usable ground/wall/ceiling **labeling** — the cheapest useful
tier. Stop here if you only need labels.

---

## Stage 3 — Discrete Plane Extraction (RANSAC, iterative)

To get plane *instances* (not just labels) — "wall #3, equation, extent":

```
remaining = voxels of target class (e.g. all VERTICAL)
planes = []
while size(remaining) > MIN_PLANE_VOXELS:        # e.g. 50
    plane = RANSAC_plane_fit(remaining,
                             dist_thresh = D,     # inlier band, e.g. 1–2 voxel sizes
                             gravity_constraint)  # see below
    if plane.inlier_count < MIN_PLANE_VOXELS: break
    planes.append(plane)
    remaining -= plane.inliers                    # remove, find next plane
```

**Gravity-constrained RANSAC (important optimization):** don't fit fully free planes. For
walls, constrain the candidate normal to be perpendicular to gravity (2-DOF: heading +
offset) instead of a free 3-DOF plane. For floors/ceilings, constrain normal parallel to
gravity (1-DOF: just height). This massively shrinks the search, speeds convergence, and
rejects spurious tilted fits. PCL's `SACMODEL_PERPENDICULAR_PLANE` / parallel-plane models
do exactly this — set the axis to the gravity vector with an angular epsilon.

Per extracted plane, store: plane equation (n, d), inlier set, and a 2D **extent** (project
inliers onto the plane, take bounding polygon / oriented bbox) so you know the wall's actual
size, not just its infinite-plane equation.

---

## Stage 4 — Floor vs. Ceiling Split

Both are HORIZONTAL; separate by height along gravity:
- Project horizontal-plane centroids onto the gravity axis.
- Lowest dominant cluster = **floor**; highest = **ceiling**; middles = tables/shelves/other
  horizontal surfaces.
- For a single-story scan there's usually one clear floor and one ceiling; use the trajectory
  height (sensor was carried at ~chest height) as a sanity anchor between them.

---

## Stage 5 (optional, high value) — Mesh Regularization

For voxels that are confident plane inliers, snap them to the fitted plane before meshing:
```
for v in plane.inliers:
    v.center_adjusted = project_point_onto_plane(v.center, plane)
```
Result: dead-flat walls/floors in the output mesh instead of noise-bumpy ones. Keep this
**optional and reversible** (store adjusted separately) so you can compare and so non-planar
detail isn't destroyed. Only snap high-confidence inliers; leave OTHER untouched.

---

## Stage 6 (optional, ties to degeneracy) — Live Constraint Signal

The set of currently-visible plane normals is a cheap geometric-constraint indicator:

```
per frame (or sliding window):
    N = set of distinct plane normal directions currently observed
    rank = number of (near-)independent directions in N   # 0,1,2,3
    if rank <= 1:  # only one direction or none → under-constrained (e.g. facing sky/open)
        emit degeneracy_warning   # trust IMU more, gate LiDAR updates
```

This is a lightweight, interpretable cousin of eigenvalue/condition-number degeneracy
detection: when you can only see one plane direction (or none — sky), translation along /
rotation about the unconstrained axes is poorly observed. It does NOT replace proper
information-matrix gating, but it's cheap, runs on data you're already computing, and is a
good first-line early warning. Treat as a heuristic flag, not ground truth.

---

## Difficulty / Risk

| Tier | What you get | Effort | Risk |
|---|---|---|---|
| Stage 1–2 | Ground/wall/ceiling **labels** | Low (days) | Low |
| Stage 3–4 | Discrete plane **instances** + extent | Moderate | Low (PCL does heavy lifting) |
| Stage 5 | Regularized flat meshes | Low-moderate | Low if inlier-gated |
| Stage 6 | Degeneracy early-warning | Low | Medium (heuristic; validate) |
| (not here) | Semantic seg (door/window/furniture) | High | ML regime — different project |

---

## Tooling
- **PCL** for normals (`NormalEstimation`) and RANSAC (`SACSegmentation`,
  `SACMODEL_PERPENDICULAR_PLANE` / parallel-plane with gravity axis + epsilon). Heavy lifting
  is library-provided.
- Work in the voxel map's coordinate frame; ensure the gravity vector is expressed in that
  same frame before classifying.

## Known Traps
- **Don't fit free 3-DOF planes** when gravity lets you constrain to 1–2 DOF. Free fits are
  slower and pick up spurious tilted planes.
- **Furniture against walls / pictures / baseboards**: "wall" = dominant plane + tolerance;
  deciding picture-frame-vs-wall is a thresholding judgment, not a hard boundary. Accept that
  the floor/wall/ceiling *skeleton* is reliable but fine clutter classification is fuzzy.
- **Gravity frame mismatch**: classifying normals against a gravity vector in the wrong frame
  silently mislabels everything. Verify frames first.
- **Over-snapping in Stage 5**: snapping non-planar detail (moldings, textured surfaces) to a
  plane destroys real geometry. Inlier-gate aggressively; keep it reversible.
- **Stage 6 as truth**: it's an early-warning heuristic, not a replacement for proper
  information-matrix degeneracy gating. Use it to *raise suspicion*, not to make hard
  estimator decisions alone.

## Definition of Done
- Occupied voxels labeled ground/wall/ceiling/other via gravity-aided normals.
- (If pursued) discrete plane instances with equations + extents extracted via
  gravity-constrained RANSAC.
- (If pursued) optional mesh regularization and live degeneracy signal, both validated and
  reversible/heuristic as noted.