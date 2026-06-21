#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import struct
import subprocess
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from livox_interfaces.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import yaml

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAS_PC2_PY = True
except Exception:
    _HAS_PC2_PY = False

_PREVIEW_MAX_POINTS = 30_000


PACKAGE_SHARE = Path(get_package_share_directory("scanner_control"))
WEB_ROOT = PACKAGE_SHARE / "web"
BRINGUP_SHARE = Path(get_package_share_directory("scanner_bringup"))
# Calibration YAML lives in the project scripts/ dir, resolved via the bringup symlink chain.
_CALIB_VIA_BRINGUP = (BRINGUP_SHARE / "config" / "point_lio_horizon.yaml").resolve()
CALIB_PATH = _CALIB_VIA_BRINGUP.parent.parent.parent.parent / "scripts" / "calib_lidar_camera.yaml"


def _cloud_bag_to_las(bag_dir: Path, out_path: Path) -> int:
    """Extract /cloud_registered PointCloud2 messages from a bag and write a LAS file."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import laspy

    PointCloud2Msg = get_message("sensor_msgs/msg/PointCloud2")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/cloud_registered"]))

    chunks: list[np.ndarray] = []
    chunks_i: list[np.ndarray] = []
    while r.has_next():
        _, data, _ = r.read_next()
        msg = deserialize_message(data, PointCloud2Msg)
        if msg.width == 0:
            continue
        offsets = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        n = msg.width * msg.height
        # Reshape to (n_points, point_step) so we can slice 4 bytes per field
        pts = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
        xyz = np.empty((n, 3), dtype=np.float32)
        for i, ax in enumerate(["x", "y", "z"]):
            off = offsets[ax]
            xyz[:, i] = np.frombuffer(pts[:, off:off + 4].tobytes(), dtype="<f4")
        # Filter out NaN/Inf and origin points
        valid = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
        chunks.append(xyz[valid])
        if "intensity" in offsets:
            off = offsets["intensity"]
            intensity = np.frombuffer(pts[:, off:off + 4].tobytes(), dtype="<f4")
            chunks_i.append(intensity[valid])
        else:
            chunks_i.append(np.zeros(valid.sum(), dtype=np.float32))

    if not chunks:
        raise RuntimeError("No /cloud_registered messages in bag")

    xyz_all = np.vstack(chunks).astype(np.float64)
    int_all = (np.concatenate(chunks_i).clip(0, 255).astype(np.uint16)) * 256

    header = laspy.LasHeader(point_format=0, version="1.4")
    header.offsets = xyz_all.mean(axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])
    las = laspy.LasData(header=header)
    las.x = xyz_all[:, 0]
    las.y = xyz_all[:, 1]
    las.z = xyz_all[:, 2]
    las.intensity = int_all
    las.write(str(out_path))
    return len(xyz_all)


def _now() -> float:
    return time.time()


class TopicRateTracker:
    def __init__(self, max_samples: int = 32):
        self._samples = deque(maxlen=max_samples)

    def tick(self, stamp: Optional[float] = None) -> None:
        self._samples.append(stamp if stamp is not None else _now())

    def rate_hz(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        duration = self._samples[-1] - self._samples[0]
        if duration <= 0.0:
            return 0.0
        return (len(self._samples) - 1) / duration

    def age_s(self) -> Optional[float]:
        if not self._samples:
            return None
        return _now() - self._samples[-1]


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    scanner_process: Optional[subprocess.Popen] = None
    run_cwd: Path = field(default_factory=Path.cwd)
    sessions_root: Optional[Path] = None  # if set, overrides run_cwd/"sessions"

    def _sessions_dir(self) -> Path:
        return self.sessions_root if self.sessions_root is not None else (self._sessions_dir())
    last_session_dir: Optional[str] = None
    latest_health: Optional[float] = None
    latest_mesh_points: list[list[float]] = field(default_factory=list)
    latest_mesh_triangles: int = 0
    latest_mesh_frame: str = "camera_init"
    latest_camera_bmp: Optional[bytes] = None
    latest_camera_shape: Optional[tuple[int, int]] = None
    latest_camera_encoding: Optional[str] = None
    latest_camera_time: Optional[float] = None
    latest_odom_time: Optional[float] = None
    latest_odom_position: Optional[list[float]] = None
    lidar_rate: TopicRateTracker = field(default_factory=TopicRateTracker)
    imu_rate: TopicRateTracker = field(default_factory=TopicRateTracker)
    camera_rate: TopicRateTracker = field(default_factory=TopicRateTracker)
    mesh_rate: TopicRateTracker = field(default_factory=TopicRateTracker)
    processing_process: Optional[subprocess.Popen] = None
    processing_cloud_record_proc: Optional[subprocess.Popen] = None
    processing_cloud_bag_tmp: Optional[str] = None
    processing_capture_dir: Optional[str] = None
    processing_mesh_path: Optional[str] = None
    processing_log_path: Optional[str] = None
    processing_status: str = "idle"
    processing_message: Optional[str] = None
    latest_preview_bytes: Optional[bytes] = None
    colorize_thread: Optional[threading.Thread] = None
    colorize_status: str = "idle"
    colorize_capture_dir: Optional[str] = None
    colorize_message: Optional[str] = None
    potree_process: Optional[subprocess.Popen] = None
    potree_session_dir: Optional[str] = None
    potree_port: int = 8087

    def refresh_process_state(self) -> None:
        with self.lock:
            proc = self.scanner_process
            if proc is not None and proc.poll() is not None:
                self.scanner_process = None
                self.last_session_dir = self._find_latest_session_dir()
            proc = self.processing_process
            if proc is not None and proc.poll() is not None and self.processing_status == "running":
                self.processing_process = None
                if self.processing_mesh_path and Path(self.processing_mesh_path).is_file():
                    self.processing_status = "completed"
                    self.processing_message = "processing finished"
                else:
                    self.processing_status = "failed"
                    self.processing_message = "processing exited without a mesh artifact"

    def _find_latest_session_dir(self) -> Optional[str]:
        sessions_root = self._sessions_dir()
        if not sessions_root.is_dir():
            return None
        session_dirs = sorted(
            (path for path in sessions_root.iterdir() if path.is_dir()),
            key=lambda path: path.stat().st_mtime,
        )
        return str(session_dirs[-1]) if session_dirs else None

    def _resolve_capture_dir(self, capture_dir: str) -> Optional[Path]:
        try:
            capture_path = Path(capture_dir).resolve()
        except Exception:
            return None
        sessions_root = (self._sessions_dir()).resolve()
        if capture_path.is_dir() and sessions_root in capture_path.parents:
            return capture_path
        return None

    def _capture_summary(self, session_dir: Path) -> dict:
        metadata_path = session_dir / "metadata.yaml"
        bag_files = sorted(session_dir.glob("*.db3"))
        bag_size = sum(path.stat().st_size for path in bag_files)
        metadata = {}
        if metadata_path.is_file():
            try:
                metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
            except Exception:
                metadata = {}

        bag_info = metadata.get("rosbag2_bagfile_information", {})
        duration_ns = bag_info.get("duration", {}).get("nanoseconds", 0)
        total_messages = int(bag_info.get("message_count", 0) or 0)
        topic_counts = {}
        for item in bag_info.get("topics_with_message_count", []):
            topic = item.get("topic_metadata", {}).get("name")
            if topic:
                topic_counts[topic] = int(item.get("message_count", 0) or 0)

        lidar_messages = topic_counts.get("/livox/lidar", 0)
        imu_messages = topic_counts.get("/livox/imu", 0)
        processed_mesh = session_dir / "mesh_dense_replay.ply"
        colored_mesh = session_dir / "mesh_colored.ply"
        log_path = session_dir / "dense_replay.log"
        pointcloud_las = session_dir / "pointcloud.las"
        potree_dir = session_dir / "potree"

        processable = True
        process_blocker = None
        if not bag_files:
            processable = False
            process_blocker = "capture has no rosbag sqlite file"
        elif duration_ns <= 0 or total_messages <= 1:
            processable = False
            process_blocker = "capture ended before LiDAR/IMU data was recorded"
        elif lidar_messages <= 0:
            processable = False
            process_blocker = "capture has no Livox LiDAR messages"
        elif imu_messages <= 0:
            processable = False
            process_blocker = "capture has no Livox IMU messages"

        return {
            "name": session_dir.name,
            "path": str(session_dir),
            "bag_size_bytes": bag_size,
            "duration_seconds": float(duration_ns) / 1_000_000_000 if duration_ns else None,
            "processed_mesh": str(processed_mesh) if processed_mesh.is_file() else None,
            "colored_mesh": str(colored_mesh) if colored_mesh.is_file() else None,
            "log_path": str(log_path) if log_path.is_file() else None,
            "mtime": session_dir.stat().st_mtime,
            "message_count": total_messages,
            "lidar_messages": lidar_messages,
            "imu_messages": imu_messages,
            "processable": processable,
            "process_blocker": process_blocker,
            "has_pointcloud": pointcloud_las.is_file(),
            "has_potree": potree_dir.is_dir(),
        }

    def _captures(self) -> list[dict]:
        sessions_root = self._sessions_dir()
        if not sessions_root.is_dir():
            return []

        captures = []
        for session_dir in sorted(
            (path for path in sessions_root.iterdir() if path.is_dir()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        ):
            captures.append(self._capture_summary(session_dir))
        return captures

    def captures_payload(self) -> dict:
        self.refresh_process_state()
        with self.lock:
            return {
                "captures": self._captures(),
                "processing": {
                    "status": self.processing_status,
                    "capture_dir": self.processing_capture_dir,
                    "mesh_path": self.processing_mesh_path,
                    "log_path": self.processing_log_path,
                    "message": self.processing_message,
                },
                "colorize": {
                    "status": self.colorize_status,
                    "capture_dir": self.colorize_capture_dir,
                    "message": self.colorize_message,
                },
                "potree": {
                    "running": self.potree_process is not None and self.potree_process.poll() is None,
                    "session_dir": self.potree_session_dir,
                    "port": self.potree_port,
                    "url": f"http://localhost:{self.potree_port}" if (
                        self.potree_process is not None and self.potree_process.poll() is None
                    ) else None,
                },
            }

    def delete_capture(self, capture_dir: str) -> tuple[bool, str]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return False, "capture path is outside sessions/"
        if not capture_path.is_dir():
            return False, "capture directory not found"
        with self.lock:
            if self.processing_capture_dir == str(capture_path):
                if self.processing_process is not None and self.processing_process.poll() is None:
                    return False, "cannot delete a capture that is currently being processed"
        try:
            shutil.rmtree(capture_path)
        except Exception as exc:
            return False, f"delete failed: {exc}"
        with self.lock:
            if self.last_session_dir == str(capture_path):
                self.last_session_dir = self._find_latest_session_dir()
        return True, f"deleted {capture_path.name}"

    def rename_capture(self, capture_dir: str, new_name: str) -> tuple[bool, str]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return False, "capture path is outside sessions/"
        if not capture_path.is_dir():
            return False, "capture directory not found"
        safe_name = "".join(c if c.isalnum() or c in "-_" else "_" for c in new_name.strip())
        if not safe_name:
            return False, "new name is empty after sanitization"
        sessions_root = (self._sessions_dir()).resolve()
        new_path = sessions_root / safe_name
        if new_path.exists():
            return False, f"a session named '{safe_name}' already exists"
        try:
            capture_path.rename(new_path)
        except Exception as exc:
            return False, f"rename failed: {exc}"
        with self.lock:
            if self.last_session_dir == str(capture_path):
                self.last_session_dir = str(new_path)
            if self.processing_capture_dir == str(capture_path):
                self.processing_capture_dir = str(new_path)
        return True, f"renamed to {safe_name}"

    def start_scan(self, session_name: Optional[str] = None) -> tuple[bool, str]:
        with self.lock:
            if self.scanner_process is not None and self.scanner_process.poll() is None:
                return False, "scan already running"

            cmd = [
                "ros2",
                "launch",
                "scanner_bringup",
                "scanner.launch.py",
                "enable_camera:=true",
                "foxglove:=false",
                "rviz:=false",
            ]
            if session_name:
                safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in session_name.strip())
                if safe:
                    cmd.append(f"session_name:={safe}")
            self.scanner_process = subprocess.Popen(
                cmd,
                cwd=str(self.run_cwd),
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
            )
            return True, f"started scan process pid {self.scanner_process.pid}"

    def stop_scan(self) -> tuple[bool, str]:
        with self.lock:
            proc = self.scanner_process
            if proc is None or proc.poll() is not None:
                self.scanner_process = None
                self.last_session_dir = self._find_latest_session_dir()
                return False, "scan is not running"

            os.killpg(proc.pid, signal.SIGINT)

        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=10)

        with self.lock:
            self.scanner_process = None
            self.last_session_dir = self._find_latest_session_dir()
        return True, "scan stopped"

    def start_processing(self, capture_dir: str) -> tuple[bool, str]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return False, "capture path is outside sessions/"
        if not capture_path.is_dir():
            return False, "capture directory not found"
        capture = self._capture_summary(capture_path)
        if not capture["processable"]:
            return False, capture["process_blocker"]

        with self.lock:
            if self.scanner_process is not None and self.scanner_process.poll() is None:
                return False, "stop the live scan before running full processing"
            if self.processing_process is not None and self.processing_process.poll() is None:
                return False, "processing is already running"

            output_mesh = capture_path / "mesh_dense_replay.ply"
            log_path = capture_path / "dense_replay.log"
            point_lio_config = BRINGUP_SHARE / "config" / "point_lio_horizon_dense.yaml"
            meshing_dense_config = BRINGUP_SHARE / "config" / "meshing_dense.yaml"
            meshing_cfg = yaml.safe_load(meshing_dense_config.read_text(encoding="utf-8")) or {}
            meshing_params = meshing_cfg.setdefault("/**", {}).setdefault("ros__parameters", {})
            meshing_params["save_path"] = str(output_mesh)
            meshing_params["save_every_publish"] = True

            temp_file = tempfile.NamedTemporaryFile("w", delete=False, suffix=".yaml", encoding="utf-8")
            with temp_file:
                yaml.safe_dump(meshing_cfg, temp_file, sort_keys=False)
            temp_config_path = Path(temp_file.name)

            cmd = [
                "ros2",
                "launch",
                "scanner_bringup",
                "scanner.launch.py",
                "use_bag:=true",
                f"bag_path:={capture_path}",
                "record:=false",
                "foxglove:=false",
                "rviz:=false",
                f"point_lio_config:={point_lio_config}",
                f"meshing_config:={temp_config_path}",
            ]
            log_handle = log_path.open("w", encoding="utf-8")
            proc = subprocess.Popen(
                cmd,
                cwd=str(self.run_cwd),
                preexec_fn=os.setsid,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
            )

            # Record /cloud_registered (already deskewed, world-frame) alongside the replay.
            # Saved to a temp bag then converted to pointcloud.las after replay finishes.
            cloud_bag_tmp = str(capture_path / "_cloud_registered_tmp")
            cloud_record_proc = subprocess.Popen(
                ["ros2", "bag", "record", "/cloud_registered", "-o", cloud_bag_tmp],
                cwd=str(self.run_cwd),
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=os.environ.copy(),
            )

            self.processing_process = proc
            self.processing_cloud_record_proc = cloud_record_proc
            self.processing_cloud_bag_tmp = cloud_bag_tmp
            self.processing_capture_dir = str(capture_path)
            self.processing_mesh_path = str(output_mesh)
            self.processing_log_path = str(log_path)
            self.processing_status = "running"
            self.processing_message = "dense replay started"

        thread = threading.Thread(
            target=self._monitor_processing,
            args=(proc, capture_path, temp_config_path, log_handle,
                  cloud_record_proc, cloud_bag_tmp),
            daemon=True,
        )
        thread.start()
        return True, f"started full processing for {capture_path.name}"

    def _monitor_processing(
        self,
        proc: subprocess.Popen,
        capture_path: Path,
        temp_config_path: Path,
        log_handle,
        cloud_record_proc: Optional[subprocess.Popen],
        cloud_bag_tmp: Optional[str],
    ) -> None:
        try:
            bag_pid = None
            deadline = _now() + 30.0
            match_term = f"ros2 bag play {capture_path}"
            while _now() < deadline and proc.poll() is None and bag_pid is None:
                result = subprocess.run(
                    ["pgrep", "-P", str(proc.pid), "-f", match_term],
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0 and result.stdout.strip():
                    bag_pid = int(result.stdout.strip().splitlines()[0])
                    break
                time.sleep(1.0)

            if bag_pid is not None:
                while proc.poll() is None:
                    if subprocess.run(["ps", "-p", str(bag_pid)], capture_output=True).returncode != 0:
                        break
                    time.sleep(2.0)
                if proc.poll() is None:
                    time.sleep(5.0)
                    os.killpg(proc.pid, signal.SIGINT)

            try:
                proc.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=15)

            # Stop the cloud recorder and convert its output to LAS
            if cloud_record_proc is not None:
                try:
                    os.killpg(cloud_record_proc.pid, signal.SIGINT)
                    cloud_record_proc.wait(timeout=10)
                except Exception:
                    pass
                if cloud_bag_tmp:
                    las_path = capture_path / "pointcloud.las"
                    try:
                        _cloud_bag_to_las(Path(cloud_bag_tmp), las_path)
                    except Exception:
                        pass  # non-fatal; mesh is the primary output
                    shutil.rmtree(cloud_bag_tmp, ignore_errors=True)

        finally:
            log_handle.close()
            try:
                temp_config_path.unlink(missing_ok=True)
            except Exception:
                pass

            with self.lock:
                self.processing_process = None
                self.processing_cloud_record_proc = None
                self.processing_cloud_bag_tmp = None
                if self.processing_mesh_path and Path(self.processing_mesh_path).is_file():
                    self.processing_status = "completed"
                    self.processing_message = "dense replay finished"
                else:
                    self.processing_status = "failed"
                    self.processing_message = "dense replay did not produce a mesh"

    def status_payload(self) -> dict:
        self.refresh_process_state()
        with self.lock:
            proc = self.scanner_process
            running = proc is not None and proc.poll() is None
            return {
                "scanner_running": running,
                "scanner_pid": proc.pid if running else None,
                "last_session_dir": self.last_session_dir,
                "health": self.latest_health,
                "camera": {
                    "available": self.latest_camera_bmp is not None,
                    "shape": self.latest_camera_shape,
                    "encoding": self.latest_camera_encoding,
                    "age_s": None if self.latest_camera_time is None else _now() - self.latest_camera_time,
                    "rate_hz": self.camera_rate.rate_hz(),
                },
                "lidar": {
                    "rate_hz": self.lidar_rate.rate_hz(),
                    "age_s": self.lidar_rate.age_s(),
                },
                "imu": {
                    "rate_hz": self.imu_rate.rate_hz(),
                    "age_s": self.imu_rate.age_s(),
                },
                "mesh": {
                    "triangle_count": self.latest_mesh_triangles,
                    "preview_point_count": len(self.latest_mesh_points),
                    "frame_id": self.latest_mesh_frame,
                    "rate_hz": self.mesh_rate.rate_hz(),
                    "age_s": self.mesh_rate.age_s(),
                },
                "odom": {
                    "age_s": None if self.latest_odom_time is None else _now() - self.latest_odom_time,
                    "position": self.latest_odom_position,
                },
                "processing": {
                    "status": self.processing_status,
                    "capture_dir": self.processing_capture_dir,
                    "mesh_path": self.processing_mesh_path,
                    "message": self.processing_message,
                },
            }

    def mesh_payload(self) -> dict:
        with self.lock:
            return {
                "frame_id": self.latest_mesh_frame,
                "triangle_count": self.latest_mesh_triangles,
                "points": self.latest_mesh_points,
            }

    def camera_payload(self) -> Optional[bytes]:
        with self.lock:
            return self.latest_camera_bmp

    def capture_mesh_path(self, capture_dir: str) -> Optional[Path]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return None
        mesh_path = capture_path / "mesh_dense_replay.ply"
        return mesh_path if mesh_path.is_file() else None

    def capture_log_path(self, capture_dir: str) -> Optional[Path]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return None
        log_path = capture_path / "dense_replay.log"
        return log_path if log_path.is_file() else None

    def capture_colored_mesh_path(self, capture_dir: str) -> Optional[Path]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return None
        p = capture_path / "mesh_colored.ply"
        return p if p.is_file() else None

    def _potree_output_dir(self, capture_path: Path) -> Optional[Path]:
        """Return the potree/ output dir if it exists, else None."""
        p = capture_path / "potree"
        return p if p.is_dir() else None

    def start_potree(self, capture_dir: str) -> tuple[bool, str]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return False, "capture path is outside sessions/"
        if not capture_path.is_dir():
            return False, "capture directory not found"

        las_path = capture_path / "pointcloud.las"
        if not las_path.exists():
            return False, "no pointcloud.las — process the session first"

        potree_dir = capture_path / "potree"
        if not potree_dir.is_dir():
            # Convert in the calling thread (fast enough — already converted LAS)
            vendor = _CALIB_VIA_BRINGUP.parent.parent.parent.parent / "vendor" / "PotreeConverter"
            converter = vendor / "PotreeConverter"
            if not converter.exists():
                return False, "PotreeConverter not found — run scripts/setup_potree.sh"
            env = os.environ.copy()
            env["LD_LIBRARY_PATH"] = str(vendor)
            result = subprocess.run(
                [str(converter), str(las_path), "-o", str(potree_dir),
                 "-p", "index", "--title", capture_path.name],
                env=env, capture_output=True, text=True,
            )
            if result.returncode != 0 or not potree_dir.is_dir():
                return False, f"PotreeConverter failed: {result.stderr[:200]}"

        with self.lock:
            # Stop any existing potree server
            if self.potree_process is not None and self.potree_process.poll() is None:
                try:
                    self.potree_process.terminate()
                    self.potree_process.wait(timeout=3)
                except Exception:
                    pass

            proc = subprocess.Popen(
                ["python3", "-m", "http.server", str(self.potree_port),
                 "--directory", str(potree_dir)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.potree_process = proc
            self.potree_session_dir = str(capture_path)

        return True, f"http://localhost:{self.potree_port}"

    def stop_potree(self) -> tuple[bool, str]:
        with self.lock:
            proc = self.potree_process
            if proc is None or proc.poll() is not None:
                self.potree_process = None
                self.potree_session_dir = None
                return False, "not running"
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                proc.kill()
            self.potree_process = None
            self.potree_session_dir = None
        return True, "stopped"

    def start_colorize(self, capture_dir: str, calib_path: Optional[Path]) -> tuple[bool, str]:
        capture_path = self._resolve_capture_dir(capture_dir)
        if capture_path is None:
            return False, "capture path is outside sessions/"
        if not capture_path.is_dir():
            return False, "capture directory not found"
        if not (capture_path / "mesh_dense_replay.ply").exists():
            return False, "no processed mesh — run full processing first"

        with self.lock:
            if self.colorize_thread is not None and self.colorize_thread.is_alive():
                return False, "colorization already running"
            self.colorize_status = "running"
            self.colorize_capture_dir = str(capture_path)
            self.colorize_message = "colorization started"

        def _run() -> None:
            from scanner_control.colorize import colorize_mesh, load_calibration  # noqa: PLC0415
            log_lines: list[str] = []

            def _log(msg: str) -> None:
                log_lines.append(msg)

            try:
                T = load_calibration(calib_path)
                colorize_mesh(capture_path, T, log=_log)
                with self.lock:
                    self.colorize_status = "completed"
                    self.colorize_message = "colorization finished"
            except Exception as exc:
                with self.lock:
                    self.colorize_status = "failed"
                    self.colorize_message = str(exc)

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        with self.lock:
            self.colorize_thread = t
        return True, f"started colorization for {capture_path.name}"


class ScannerControlNode(Node):
    def __init__(self, state: SharedState):
        super().__init__("scanner_control_server")
        self._state = state
        self._mesh_stride = 6
        self._mesh_point_cap = 12000

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, "/camera/d435i/color/image_raw", self._on_image, 1)
        self.create_subscription(Marker, "/scanner/mesh", self._on_mesh, 1)
        self.create_subscription(Float32, "/scanner/health", self._on_health, 10)
        self.create_subscription(Odometry, "/aft_mapped_to_init", self._on_odom, 10)
        self.create_subscription(CustomMsg, "/livox/lidar", self._on_lidar, 10)
        self.create_subscription(Imu, "/livox/imu", self._on_imu, 20)
        self.create_subscription(PointCloud2, "/cloud_registered", self._on_cloud, sensor_qos)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not _HAS_PC2_PY:
            return
        point_rows = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not point_rows:
            return
        first = point_rows[0]
        if isinstance(first, np.void) and hasattr(first, "dtype") and first.dtype.names:
            pts = np.array([(r["x"], r["y"], r["z"]) for r in point_rows], dtype=np.float32)
        else:
            pts = np.array(point_rows, dtype=np.float32)
        if pts.ndim == 2 and pts.shape[1] >= 3:
            pts = pts[:, :3]
        if len(pts) > _PREVIEW_MAX_POINTS:
            stride = max(1, len(pts) // _PREVIEW_MAX_POINTS)
            pts = pts[::stride][:_PREVIEW_MAX_POINTS]
        preview_bytes = pts.flatten().astype(np.float32).tobytes()
        with self._state.lock:
            self._state.latest_preview_bytes = preview_bytes

    def _on_lidar(self, msg: CustomMsg) -> None:
        self._state.lidar_rate.tick()

    def _on_imu(self, _msg) -> None:
        self._state.imu_rate.tick()

    def _on_health(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.latest_health = float(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        with self._state.lock:
            self._state.latest_odom_time = _now()
            pos = msg.pose.pose.position
            self._state.latest_odom_position = [float(pos.x), float(pos.y), float(pos.z)]

    def _on_mesh(self, msg: Marker) -> None:
        preview_points: list[list[float]] = []
        if msg.action == Marker.DELETE:
            with self._state.lock:
                self._state.latest_mesh_points = []
                self._state.latest_mesh_triangles = 0
                self._state.latest_mesh_frame = msg.header.frame_id or "camera_init"
            return

        raw_points = msg.points[:: self._mesh_stride] if self._mesh_stride > 1 else msg.points
        if len(raw_points) > self._mesh_point_cap:
            raw_points = raw_points[: self._mesh_point_cap]

        for pt in raw_points:
            preview_points.append([float(pt.x), float(pt.y), float(pt.z)])

        with self._state.lock:
            self._state.latest_mesh_points = preview_points
            self._state.latest_mesh_triangles = len(msg.points) // 3
            self._state.latest_mesh_frame = msg.header.frame_id or "camera_init"
            self._state.mesh_rate.tick()

    def _on_image(self, msg: Image) -> None:
        bmp = image_msg_to_bmp(msg)
        if bmp is None:
            return
        with self._state.lock:
            self._state.latest_camera_bmp = bmp
            self._state.latest_camera_shape = (msg.width, msg.height)
            self._state.latest_camera_encoding = msg.encoding
            self._state.latest_camera_time = _now()
            self._state.camera_rate.tick()


def image_msg_to_bmp(msg: Image) -> Optional[bytes]:
    if msg.encoding not in {"rgb8", "bgr8"}:
        return None

    width = int(msg.width)
    height = int(msg.height)
    if width <= 0 or height <= 0:
        return None

    row_stride = width * 3
    if msg.step < row_stride:
        return None

    rows = []
    for row_idx in range(height):
        start = row_idx * msg.step
        row = bytearray(msg.data[start : start + row_stride])
        if msg.encoding == "rgb8":
            for px in range(0, len(row), 3):
                row[px], row[px + 2] = row[px + 2], row[px]
        rows.append(bytes(row))

    padding = (4 - (row_stride % 4)) % 4
    pixel_rows = []
    for row in reversed(rows):
        pixel_rows.append(row + (b"\x00" * padding))
    pixel_data = b"".join(pixel_rows)

    file_size = 14 + 40 + len(pixel_data)
    header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, 54)
    dib = struct.pack(
        "<IIIHHIIIIII",
        40,
        width,
        height,
        1,
        24,
        0,
        len(pixel_data),
        2835,
        2835,
        0,
        0,
    )
    return header + dib + pixel_data


def make_handler(state: SharedState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            if parsed.path == "/":
                self._serve_file(WEB_ROOT / "index.html", "text/html; charset=utf-8")
                return
            if parsed.path == "/api/status":
                self._serve_json(state.status_payload())
                return
            if parsed.path == "/api/mesh":
                self._serve_json(state.mesh_payload())
                return
            if parsed.path == "/api/captures":
                self._serve_json(state.captures_payload())
                return
            if parsed.path == "/api/camera.bmp":
                payload = state.camera_payload()
                if payload is None:
                    self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera frame unavailable")
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/bmp")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
                return
            if parsed.path == "/api/capture/mesh":
                capture_dir = query.get("capture_dir", [None])[0]
                colored = query.get("colored", ["0"])[0] == "1"
                if not isinstance(capture_dir, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir is required")
                    return
                if colored:
                    mesh_path = state.capture_colored_mesh_path(capture_dir)
                    if mesh_path is None:
                        self.send_error(HTTPStatus.NOT_FOUND, "colored mesh not found — run colorization first")
                        return
                else:
                    mesh_path = state.capture_mesh_path(capture_dir)
                    if mesh_path is None:
                        self.send_error(HTTPStatus.NOT_FOUND, "processed mesh not found")
                        return
                self._serve_file(mesh_path, "application/octet-stream")
                return
            if parsed.path == "/api/capture/log":
                capture_dir = query.get("capture_dir", [None])[0]
                if not isinstance(capture_dir, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir is required")
                    return
                log_path = state.capture_log_path(capture_dir)
                if log_path is None:
                    self.send_error(HTTPStatus.NOT_FOUND, "processing log not found")
                    return
                self._serve_file(log_path, "text/plain; charset=utf-8")
                return
            if parsed.path == "/api/preview.bin":
                with state.lock:
                    preview = state.latest_preview_bytes
                if preview is None:
                    self.send_response(HTTPStatus.NO_CONTENT)
                    self.end_headers()
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(preview)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(preview)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/api/scan/start":
                payload = self._read_json()
                session_name = payload.get("session_name") or None
                ok, message = state.start_scan(session_name=session_name)
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/scan/stop":
                ok, message = state.stop_scan()
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/process/start":
                payload = self._read_json()
                capture_dir = payload.get("capture_dir")
                if not isinstance(capture_dir, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir is required")
                    return
                ok, message = state.start_processing(capture_dir)
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/capture/delete":
                payload = self._read_json()
                capture_dir = payload.get("capture_dir")
                if not isinstance(capture_dir, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir is required")
                    return
                ok, message = state.delete_capture(capture_dir)
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/capture/rename":
                payload = self._read_json()
                capture_dir = payload.get("capture_dir")
                new_name = payload.get("new_name")
                if not isinstance(capture_dir, str) or not isinstance(new_name, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir and new_name are required")
                    return
                ok, message = state.rename_capture(capture_dir, new_name)
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/capture/colorize":
                payload = self._read_json()
                capture_dir = payload.get("capture_dir")
                if not isinstance(capture_dir, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir is required")
                    return
                calib_path = CALIB_PATH if CALIB_PATH.exists() else None
                ok, message = state.start_colorize(capture_dir, calib_path)
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/capture/potree/start":
                payload = self._read_json()
                capture_dir = payload.get("capture_dir")
                if not isinstance(capture_dir, str):
                    self.send_error(HTTPStatus.BAD_REQUEST, "capture_dir is required")
                    return
                ok, message = state.start_potree(capture_dir)
                self._serve_json({"ok": ok, "message": message, "url": message if ok else None},
                                 status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/potree/stop":
                ok, message = state.stop_potree()
                self._serve_json({"ok": ok, "message": message})
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def log_message(self, format: str, *args) -> None:  # noqa: A003
            return

        def _serve_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _serve_file(self, path: Path, content_type: str) -> None:
            size = path.stat().st_size
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(size))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            with path.open("rb") as fh:
                while True:
                    chunk = fh.read(1 << 20)  # 1 MiB chunks
                    if not chunk:
                        break
                    self.wfile.write(chunk)

        def _read_json(self) -> dict:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0:
                return {}
            body = self.rfile.read(content_length)
            try:
                return json.loads(body.decode("utf-8"))
            except json.JSONDecodeError:
                return {}

    return Handler


def spin_node(node: Node) -> None:
    rclpy.spin(node)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the scanner browser control surface.")
    parser.add_argument("--bind", default="0.0.0.0", help="HTTP bind address.")
    parser.add_argument("--port", type=int, default=8090, help="HTTP port.")
    parser.add_argument(
        "--sessions-dir",
        default="",
        help="Absolute path to the sessions directory. "
             "If omitted, defaults to sessions/ inside the current working directory.",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    if args.sessions_dir.strip():
        sessions_root = Path(args.sessions_dir).expanduser().resolve()
        sessions_root.mkdir(parents=True, exist_ok=True)
        run_cwd = sessions_root.parent
    else:
        sessions_root = None
        run_cwd = Path.cwd()
    state = SharedState(run_cwd=run_cwd, sessions_root=sessions_root)
    node = ScannerControlNode(state)
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer((args.bind, args.port), make_handler(state))
    try:
        print(f"Serving scanner control UI at http://{args.bind}:{args.port}")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        try:
            state.stop_scan()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2)


if __name__ == "__main__":
    main()
