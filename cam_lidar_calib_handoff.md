# Handoff: Verify & Redo Camera→LiDAR Extrinsic Calibration (Intel D435 + Livox Horizon)

## Why this exists

The colored mesh is a blotchy mess. Before building a voxel color-confidence system, we
must confirm the camera→LiDAR mapping is actually correct, because no downstream color
cleverness fixes a wrong extrinsic — it just places wrong-location color confidently.

Two things to know going in:
1. We previously ran the Livox SDK calibration but **the image came out dark** — that run
   is suspect and must be redone (see "Dark Image Fix" below).
2. We gave the pipeline the **mounting dimensions** (the ~10mm offset). Mounting dimensions
   give **translation only**. The thing that usually wrecks projection is **rotation** —
   whether the camera optical axis is truly parallel to the LiDAR axes, or pitched/yawed by
   a degree or two. A nominal mount offset does not capture rotation. We need a measured
   full 6-DOF extrinsic (rotation + translation).

---

## Part 1 — Isolate the failure BEFORE recalibrating (10-minute test)

Don't debug on the moving final mesh — every error source produces the same blotchy
symptom there. Collapse the problem with a **single static frame** test.

1. Put the rig **completely still**.
2. Capture **one** LiDAR sweep + **one** RGB frame.
3. Project that single frame's color onto that single sweep using the current extrinsic.

Interpret:

- **Static projection is misaligned** (color sitting off geometry — a window's color landing
  on the wall beside it): problem is **static → extrinsic or intrinsic calibration**. Motion
  is NOT your issue. Proceed to recalibrate (Part 2).
- **Static projection is clean, only the moving mesh is blotchy**: calibration is fine. The
  problem is **temporal** — projecting color at the wrong trajectory pose (timestamp/interp)
  or rolling shutter, OR the coloring tool simply has no outlier rejection (last-write-wins).
  → Skip recalibration; build the voxel color-confidence system instead.

### Sub-split if static is misaligned
Project onto a scene with a straight edge (doorway, poster):
- Color **shifted but straight** → **extrinsic** (rotation/translation). Recalibrate.
- Color **warped/curved** vs. straight geometry → **intrinsic/distortion**. Re-verify the
  D435 RGB intrinsics + distortion coefficients (don't trust nominal pinhole; pull
  calibrated intrinsics, including distortion, from the camera or a checkerboard calib).

**Hunch:** given "we gave it the mounting dimensions," the most likely bug is a missing
**rotation** in the extrinsic. Expect the static test to show misalignment and the fix to
be a proper full-6-DOF calibration. But test rather than trust the hunch.

---

## Part 2 — Dark Image Fix (why the last Livox calib run failed)

The previous calibration's dark image almost certainly means the calibration target wasn't
properly exposed/visible. Calibration tools need to clearly detect a target (checkerboard /
specific pattern) in BOTH the camera image AND the LiDAR reflectivity/intensity data. A dark
image = the detector can't find target corners = garbage or failed extrinsic.

Checklist before re-running:
- **Lighting:** bright, even, diffuse illumination on the target. No harsh shadows, no
  backlighting. The target must be clearly visible in the RGB frame.
- **D435 exposure:** disable RGB auto-exposure if it's hunting; set a **fixed exposure/gain**
  that yields a well-lit (not blown-out) target. A dark frame often means auto-exposure
  metered for a bright background and crushed the target.
- **Target quality:** flat, rigid (foam-board mount, not floppy paper), high-contrast,
  correct printed dimensions. Wrinkles/curl break corner detection.
- **Target placement:** within good range of BOTH sensors (not too close for the Livox min
  range, not so far the camera can't resolve the pattern). Fill a good fraction of the frame.
- **Coverage:** capture the target at multiple distances, angles, and image positions
  (center + corners). A single pose under-constrains the solve.

---

## Part 3 — Re-run the Calibration

### Option A — Livox official calibration (what you ran before)
Re-run the Livox SDK / Livox-camera calibration tool with the dark-image issues above fixed.
Confirm which exact tool/repo was used and that it targets the Horizon (SDK1). Validate the
output extrinsic immediately with the static-frame test from Part 1 — do not assume success.

### Option B — direct_visual_lidar_calibration (recommended, robust, target-less option available)
`koide3/direct_visual_lidar_calibration` is a well-regarded LiDAR↔camera extrinsic tool that
supports an automatic/target-less workflow and works with various LiDAR + camera combos. It
optimizes the full 6-DOF transform by maximizing alignment between the camera image and the
LiDAR intensity/geometry. Good when target-based runs keep failing or you want a cross-check.

### Option C — checkerboard target-based (classic)
Standard OpenCV-style checkerboard detection in the camera + plane/board detection in the
LiDAR, solve PnP / point-to-plane for the transform. More manual but well-understood.

Whichever you use, the output is **T_lidar_cam** (4×4 rigid transform): rotation R + the
translation that should come out near your ~10mm mount offset on the relevant axis. If the
solved translation is wildly off from the known mount geometry, the solve is bad — re-run.

---

## Part 4 — Validate the New Extrinsic (don't skip)

1. **Static-frame re-test:** repeat Part 1 with the new extrinsic. Color should now sit
   crisply on geometry; straight edges colored straight; window-color on windows.
2. **Translation sanity:** solved translation ≈ known mount offset (~10mm on the mount axis).
   Large disagreement = bad solve.
3. **Reprojection error:** if the tool reports it, check it's within expected bounds
   (sub-pixel to low single-digit pixels for a good calib).
4. **Multi-angle check:** project a few static frames of the same object from different
   viewpoints; the same surface point should receive consistent color across views. Drift
   across views = residual rotation error.

Only after the static projection is crisp should the colored result be trusted on moving
scans — and only then is it worth layering the voxel color-confidence system on top.

---

## Outputs / Definition of Done
- Verified **T_lidar_cam** (full rotation + translation), stored in the pipeline config.
- Verified D435 RGB **intrinsics + distortion** (not nominal).
- Static single-frame projection is visibly crisp.
- Documented which tool produced the extrinsic and its reported error, for reproducibility.

## Quick Reference — likely cause ranking for the blotchy mesh
1. **Missing/!wrong extrinsic rotation** (mount gave translation only) — most likely.
2. Coloring tool has no outlier rejection (last-write-wins) — likely contributor.
3. Wrong-pose temporal projection (image not at its own timestamp) on moving scans.
4. Uncalibrated intrinsics/distortion.
5. No occlusion handling (foreground color on background).