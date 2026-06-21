# Scanner bringup launch/config mismatches caused live scanning, replay reconstruction, and control-surface validation to fail until the ROS wiring was corrected

**Date:** 2026-06-21  
**Component:** `src/scanner_bringup/launch/scanner.launch.py` — `_launch_setup`; `src/scanner_meshing/scanner_meshing/vdb_meshing_node.py` — `_on_cloud`, `destroy_node`, `main`; `src/scanner_control/scanner_control/control_server.py` — `main`, `SharedState.start_scan`, `SharedState.stop_scan`; `src/scanner_coverage/scanner_coverage/coverage_node.py` — `main`  
**Severity:** High — the scanner stack could appear partially alive while core functions such as Point-LIO bringup, bag replay, meshing, or the control UI failed, which blocks both real scans and validation.

---

## Observed symptom

Multiple parts of the handheld scanner stack failed during end-to-end validation on the actual box:

- the integrated `scanner.launch.py` path failed even though individual components worked in isolation
- the Livox driver and Point-LIO aborted when JSON/plain YAML files were passed as ROS 2 parameter files
- the top-level launch referenced a non-existent `point_lio.launch.py`
- the meshing node crashed on live `PointCloud2` structured dtypes
- bag replay used the wrong `ros2 bag play` argument order, so replay never actually started
- the first-pass browser control surface started but immediately died because ROS launch injected `--ros-args`
- the live preview UX was misleading because the mesh view re-centered itself every refresh

The net effect was that “working” subsystems still did not compose into a reliable live scanner or replay workflow until the launch/config/runtime path was corrected as a whole.

---

## Root cause

### Bringup mixed native config files and ROS 2 parameter files

The Livox SDK1 driver expects its Horizon settings through `user_config_path`, while Point-LIO, meshing, and coverage expect valid ROS 2 parameter YAML. The initial integrated bringup passed incompatible files directly as `--params-file` inputs:

```python
parameters=[
    PathJoinSubstitution([bringup_share, "config", "horizon.json"]),
    {"xfer_format": 1},
]
```

That made launch composition fail even though the individual components could still be brought up manually with the correct entrypoints.

### Top-level launch assumed upstream entrypoints that were not present in the vendored workspace

The bringup originally tried to include a `point_lio.launch.py` file that the installed `point_lio` package did not ship. The actual vendored package exposed the `pointlio_mapping` executable and separate sensor-specific launch files.

### Runtime data-shape assumptions were invalid for the live Horizon cloud

The meshing node assumed `sensor_msgs_py.point_cloud2.read_points()` would always yield tuples coercible to a simple `float64` array. On the live `cloud_registered` stream, it returned structured rows, causing a conversion crash:

```python
points = np.asarray(
    list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
    dtype=np.float64,
)
```

### Replay and control-surface flows had integration-specific argument handling bugs

Replay used `ros2 bag play --clock <bag_path>` instead of `ros2 bag play <bag_path> --clock`, so the replay process never actually consumed the bag. Separately, the control server used `argparse.parse_args()`, so ROS launch's injected `--ros-args` caused the process to exit before it could serve the UI.

---

## Troubleshooting steps taken

1. **Validated individual sensors and components separately** — the Horizon driver, RealSense camera, and workspace bootstrap were each exercised outside the top-level bringup, which ruled out a general machine-setup failure and narrowed the problem to launch/runtime integration.

2. **Inspected installed vendored packages rather than assuming upstream launch names** — the installed `point_lio` share tree and launch files were checked directly, which ruled in a bad bringup include rather than a missing build artifact.

3. **Ran the full stack and read process-specific failures from logs/stdout** — the Livox driver JSON parse failure, Point-LIO YAML parse failure, meshing dtype crash, replay argument-order failure, and control-panel `--ros-args` crash were each reproduced and confirmed from real process output.

4. **Replayed the saved room-walk bag with alternate configs** — this ruled out “bad scan only” as the sole explanation for sparse output and showed that the replay/meshing path itself needed a denser profile plus a clean shutdown/export path.

---

## Fix

### `src/scanner_bringup/launch/scanner.launch.py` — corrected launch wiring for live and replay paths

The bringup now:

- passes the Livox Horizon file via `user_config_path`
- starts Point-LIO via the real `pointlio_mapping` executable
- accepts configurable Point-LIO and meshing parameter files
- uses the correct `ros2 bag play <bag> --clock` argument order

```python
driver = Node(
    package="livox_ros2_driver",
    executable="livox_ros2_driver_node",
    parameters=[{
        "user_config_path": PathJoinSubstitution([bringup_share, "config", "horizon.json"]),
        "xfer_format": 1,
    }],
)

bag_replay = ExecuteProcess(
    cmd=["ros2", "bag", "play", bag_path, "--clock"],
    ...
)
```

### `src/scanner_bringup/config/*.yaml` — normalized node configs into valid ROS 2 parameter files

Point-LIO, meshing, and coverage config files were converted into proper ROS 2 parameter documents instead of plain nested YAML blobs. This removed launch-time parse failures and made config overrides safe.

### `src/scanner_meshing/scanner_meshing/vdb_meshing_node.py` — fixed live point unpacking and export stability

The mesher now handles structured `PointCloud2` rows explicitly before converting to `float64`, and shutdown no longer loses the mesh export because of double-shutdown/interrupt timing.

```python
point_rows = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
if isinstance(first_row, np.void) and first_row.dtype.names:
    points = np.asarray([(row["x"], row["y"], row["z"]) for row in point_rows], dtype=np.float64)
```

### `src/scanner_control/scanner_control/control_server.py` and `src/scanner_control/web/index.html` — made the browser control panel launchable and usable

The control server now tolerates ROS launch argument injection with `parse_known_args()`, and the LiDAR preview no longer re-centers every refresh. A manual **Reset view** action replaced constant camera resets, and the preview applies a ROS FLU → browser-view transform so the scene orientation is usable.

---

## Files changed

- `src/scanner_bringup/launch/scanner.launch.py` — `_launch_setup`, replay command wiring, config overrides
- `src/scanner_bringup/config/point_lio_horizon.yaml` — ROS 2 parameter-file normalization
- `src/scanner_bringup/config/meshing.yaml` — ROS 2 parameter-file normalization
- `src/scanner_bringup/config/coverage.yaml` — ROS 2 parameter-file normalization
- `src/scanner_bringup/config/point_lio_horizon_dense.yaml` — denser replay reconstruction profile
- `src/scanner_bringup/config/meshing_dense.yaml` — denser replay meshing profile
- `src/scanner_meshing/scanner_meshing/vdb_meshing_node.py` — live point unpacking, shutdown/export handling
- `src/scanner_coverage/scanner_coverage/coverage_node.py` — shutdown handling
- `src/scanner_control/scanner_control/control_server.py` — ROS-arg tolerant startup, scan child-process management
- `src/scanner_control/web/index.html` — control panel UI, non-resetting preview, orientation fix
- `README.md` — control-surface and dense replay documentation
- `docs/SETUP.md` — control-surface and dense replay setup instructions
- `HANDOFF.md` — current state of replay/control-surface features
