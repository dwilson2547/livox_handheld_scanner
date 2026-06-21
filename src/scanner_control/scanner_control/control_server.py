#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from livox_interfaces.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


PACKAGE_SHARE = Path(get_package_share_directory("scanner_control"))
WEB_ROOT = PACKAGE_SHARE / "web"


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

    def refresh_process_state(self) -> None:
        with self.lock:
            proc = self.scanner_process
            if proc is not None and proc.poll() is not None:
                self.scanner_process = None
                self.last_session_dir = self._find_latest_session_dir()

    def _find_latest_session_dir(self) -> Optional[str]:
        sessions_root = self.run_cwd / "sessions"
        if not sessions_root.is_dir():
            return None
        session_dirs = sorted(
            (path for path in sessions_root.iterdir() if path.is_dir() and path.name.startswith("session_")),
            key=lambda path: path.stat().st_mtime,
        )
        return str(session_dirs[-1]) if session_dirs else None

    def start_scan(self) -> tuple[bool, str]:
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


class ScannerControlNode(Node):
    def __init__(self, state: SharedState):
        super().__init__("scanner_control_server")
        self._state = state
        self._mesh_stride = 6
        self._mesh_point_cap = 12000

        self.create_subscription(Image, "/camera/d435i/color/image_raw", self._on_image, 1)
        self.create_subscription(Marker, "/scanner/mesh", self._on_mesh, 1)
        self.create_subscription(Float32, "/scanner/health", self._on_health, 10)
        self.create_subscription(Odometry, "/aft_mapped_to_init", self._on_odom, 10)
        self.create_subscription(CustomMsg, "/livox/lidar", self._on_lidar, 10)
        self.create_subscription(Imu, "/livox/imu", self._on_imu, 20)

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
            if parsed.path == "/":
                self._serve_file(WEB_ROOT / "index.html", "text/html; charset=utf-8")
                return
            if parsed.path == "/api/status":
                self._serve_json(state.status_payload())
                return
            if parsed.path == "/api/mesh":
                self._serve_json(state.mesh_payload())
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
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/api/scan/start":
                ok, message = state.start_scan()
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parsed.path == "/api/scan/stop":
                ok, message = state.stop_scan()
                self._serve_json({"ok": ok, "message": message}, status=HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
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
            body = path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def spin_node(node: Node) -> None:
    rclpy.spin(node)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the scanner browser control surface.")
    parser.add_argument("--bind", default="0.0.0.0", help="HTTP bind address.")
    parser.add_argument("--port", type=int, default=8090, help="HTTP port.")
    args, _ = parser.parse_known_args()

    rclpy.init()
    state = SharedState(run_cwd=Path.cwd())
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
