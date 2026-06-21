#!/usr/bin/env python3
"""
coverage_node
=============
Tracks per-voxel observation density and a LIO health score, and republishes
both for the operator viewer (Foxglove/RViz).

This is the heart of the "don't wave an invisible laser around blind" UX
requirement: the operator sees a live heatmap of what's been covered and an
at-a-glance tracking-health indicator.

STATUS: partial. The coverage heatmap path is implemented (hash-voxel hit
counting -> PointCloud2 with intensity = observation count). The health path
has a working FALLBACK (odom covariance trace) but the "real" health signal
should come from Point-LIO internals -- see TODO(copilot) and HANDOFF §5.
"""
import struct
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Header
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry

try:
    # sensor_msgs_py ships with ROS 2 Humble desktop
    from sensor_msgs_py import point_cloud2 as pc2
    _HAS_PC2_PY = True
except Exception:  # pragma: no cover
    _HAS_PC2_PY = False


def _voxel_key(x, y, z, inv_size):
    return (int(x * inv_size), int(y * inv_size), int(z * inv_size))


class CoverageNode(Node):
    def __init__(self):
        super().__init__("coverage")

        # --- params ---
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("map_frame", "camera_init")
        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("observations_for_full", 8)
        self.declare_parameter("heatmap_topic", "/scanner/coverage")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("health_topic", "/scanner/health")
        self.declare_parameter("odom_cov_trace_warn", 0.05)

        self.map_frame = self.get_parameter("map_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.inv_size = 1.0 / self.voxel_size
        self.obs_full = int(self.get_parameter("observations_for_full").value)
        self.cov_trace_warn = float(self.get_parameter("odom_cov_trace_warn").value)

        # voxel_key -> observation count
        self.voxels = defaultdict(int)

        # --- I/O ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self._on_cloud,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            10,
        )
        self.heatmap_pub = self.create_publisher(
            PointCloud2, self.get_parameter("heatmap_topic").value, 1
        )
        self.health_pub = self.create_publisher(
            Float32, self.get_parameter("health_topic").value, 10
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish_heatmap)

        self.get_logger().info(
            f"coverage up: voxel={self.voxel_size}m, full@{self.obs_full} obs"
        )
        if not _HAS_PC2_PY:
            self.get_logger().warn(
                "sensor_msgs_py not found; install ros-humble-sensor-msgs-py"
            )

    # ----------------------------------------------------------------- cloud
    def _on_cloud(self, msg: PointCloud2):
        if not _HAS_PC2_PY:
            return
        inv = self.inv_size
        v = self.voxels
        # read_points is a generator of (x, y, z, ...) tuples
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            v[_voxel_key(p[0], p[1], p[2], inv)] += 1

    # ------------------------------------------------------------------ odom
    def _on_odom(self, msg: Odometry):
        # FALLBACK health: trace of position covariance -> [0,1] health score.
        # TODO(copilot): replace with Point-LIO's real degeneracy/condition-number
        # signal once the fork exposes it. See HANDOFF §5.
        cov = msg.pose.covariance  # 6x6 row-major
        trace = cov[0] + cov[7] + cov[14]  # xx + yy + zz
        # map trace -> health in [0,1]; trace==0 -> perfect, >=warn -> degraded
        if self.cov_trace_warn <= 0:
            health = 1.0
        else:
            health = max(0.0, 1.0 - (trace / self.cov_trace_warn))
        self.health_pub.publish(Float32(data=float(health)))

    # --------------------------------------------------------------- heatmap
    def _publish_heatmap(self):
        if not self.voxels or not _HAS_PC2_PY:
            return
        inv_full = 1.0 / max(1, self.obs_full)
        points = []
        for (ix, iy, iz), count in self.voxels.items():
            x = (ix + 0.5) * self.voxel_size
            y = (iy + 0.5) * self.voxel_size
            z = (iz + 0.5) * self.voxel_size
            intensity = min(1.0, count * inv_full)  # 0..1 coverage completeness
            points.append((x, y, z, intensity))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = pc2.create_cloud(header, fields, points)
        self.heatmap_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
