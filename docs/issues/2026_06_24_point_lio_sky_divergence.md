# Point-LIO diverges silently when LiDAR is pointed at open sky, producing no mesh

**Date:** 2026-06-24  
**Component:** `src/scanner_bringup/launch/scanner.launch.py` — dense replay pipeline; Point-LIO iEKF estimator  
**Severity:** High — renders any outdoor scan with upward LiDAR orientation completely unprocessable; no mesh, no colorization, no usable output

---

## Observed symptom

During an outdoor car scan (`session_20260624_014308`, 140s, 6.7 GB bag), the dense replay processing completed without error but produced no mesh and only 24,371 points in the LAS output (expected: millions). The control panel UI showed the fusion frozen on the last valid frame with no further updates. Processing reported `"dense replay did not produce a mesh"`. The raw bag was confirmed intact — LiDAR messages present at 10 Hz for the full 140 seconds with a maximum inter-message gap of 0.131 s.

---

## Root cause

### Insufficient geometric features from open-sky LiDAR orientation

The Livox Horizon was inadvertently pointed toward open sky during part of the scan. Open sky returns few or no LiDAR points — the beam diverges without hitting a surface. Point-LIO's iEKF formulation requires a continuous stream of point-to-plane residuals from the incoming cloud to maintain its state estimate. When the point cloud degenerates (near-zero returns), the estimator loses observability constraints and diverges.

### Silent failure — no error, no diagnostic output

Point-LIO does not emit any warning when its input cloud is degenerate or when the estimator diverges. The last successfully fused frame stays rendered in the UI indefinitely, making it appear as though the system is merely slow rather than failed. Processing continues to completion; the mesh writer (`vdb_meshing_node`) simply has no surface to write and exits without error. The control server then reports failure only via the absence of the expected output file.

---

## Troubleshooting steps taken

1. **Checked raw bag message counts and timestamps** — 1,401 LiDAR messages over 140 s at 10 Hz with max gap 0.131 s. LiDAR data ruled out as the cause; the bag is complete.

2. **Checked IMU and camera topics** — both continuous at expected rates (139.5 Hz IMU, 30 Hz camera). No sensor dropout during capture.

3. **Inspected dense replay log** — Point-LIO IMU initialization reached 100 %, then the bag player exited cleanly. No mapping-phase log lines appeared after init, confirming divergence occurred immediately after initialization completed.

4. **Checked LAS output** — 24,371 points over a ~39 × 26 m bounding box. Consistent with a handful of LiDAR frames captured during the brief initialization window before divergence, not with tracked motion across the scene.

5. **User confirmed LiDAR was pointed at sky** — consistent with all findings above.

---

## Fix

### `src/scanner_control/scanner_control/control_server.py` — live point density tracking

Added `lidar_points_samples` (rolling deque, 16 frames) to `SharedState`. `_on_lidar` now
appends `msg.point_num` on each CustomMsg. `status_payload` exposes `lidar.points_per_frame`
(rolling mean) and `lidar.sparse_warning` (true when rate > 0 and mean < 5,000 pts/frame).

### `src/scanner_control/scanner_control/control_server.py` — replay divergence detection

Added `cloud_registered_rate: TopicRateTracker` to `SharedState`, reset at the start of each
processing run. `_on_cloud` ticks this tracker. `_monitor_processing` checks every 2 s: if
`/cloud_registered` was ever active and then stalls for >5 s while replay is still running,
`processing_message` is updated to flag the divergence. The UI renders this in orange.

### `src/scanner_control/web/index.html` — warning UI

Added a **LiDAR pts/frame** metric to the status grid. Added an amber warning banner
(`#sparseWarning`) shown when `lidar.sparse_warning` is true. Processing status text turns
orange when the message contains "diverged".

### `src/scanner_control/scanner_control/control_server.py` — Potree URL fix (discovered during same session)

Potree links were hardcoded to `localhost:8087`, breaking access from any machine other than
the NUC. Added `_client_host()` and `_fix_potree_url()` helpers to the HTTP handler; all
potree URL responses now substitute the client's Host header IP.

---

## Files changed

- `src/scanner_control/scanner_control/control_server.py` — `SharedState`, `_on_lidar`, `_on_cloud`, `start_processing`, `_monitor_processing`, `status_payload`, `make_handler`
- `src/scanner_control/web/index.html` — status grid, sparse warning banner, processing status colour
