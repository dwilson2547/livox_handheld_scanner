#!/usr/bin/env python3
"""
vdb_meshing_node
================
Wraps VDBFusion to incrementally integrate the LIO-registered point cloud into
a TSDF volume and publish a live triangle mesh for the operator viewer.

STATUS: STUB. The ROS plumbing (subscriptions, params, timer, mesh publish
skeleton) is here. The actual VDBFusion integration calls are marked
TODO(copilot). See HANDOFF §6.

Why VDBFusion: sparse-volume TSDF, fast incremental integration, gives a real
surface (not just points) so the operator can judge coverage holes, and the
same volume can be re-integrated offline at finer voxel size for the print mesh.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAS_PC2_PY = True
except Exception:
    _HAS_PC2_PY = False

# TODO(copilot): `pip install vdbfusion` (or build from source) and import here.
# from vdbfusion import VDBVolume
_HAS_VDB = False


class VdbMeshingNode(Node):
    def __init__(self):
        super().__init__("vdb_meshing")

        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("map_frame", "camera_init")
        self.declare_parameter("voxel_size", 0.02)
        self.declare_parameter("sdf_trunc", 0.08)
        self.declare_parameter("space_carving", True)
        self.declare_parameter("mesh_every_n_clouds", 20)
        self.declare_parameter("min_weight", 2.0)
        self.declare_parameter("mesh_topic", "/scanner/mesh")
        self.declare_parameter("save_on_shutdown", True)
        self.declare_parameter("save_path", "sessions/mesh_live.ply")

        self.map_frame = self.get_parameter("map_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.sdf_trunc = float(self.get_parameter("sdf_trunc").value)
        self.space_carving = bool(self.get_parameter("space_carving").value)
        self.mesh_every_n = int(self.get_parameter("mesh_every_n_clouds").value)
        self.min_weight = float(self.get_parameter("min_weight").value)

        self._last_origin = (0.0, 0.0, 0.0)  # sensor origin for TSDF carving
        self._cloud_count = 0

        # TODO(copilot): instantiate the TSDF volume.
        # self.vdb = VDBVolume(self.voxel_size, self.sdf_trunc, self.space_carving)
        self.vdb = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("cloud_topic").value, self._on_cloud, sensor_qos
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10
        )
        self.mesh_pub = self.create_publisher(
            Marker, self.get_parameter("mesh_topic").value, 1
        )

        self.get_logger().info(
            f"vdb_meshing up: voxel={self.voxel_size}m trunc={self.sdf_trunc}m"
        )
        if not _HAS_VDB:
            self.get_logger().warn(
                "VDBFusion not available -- meshing is a NO-OP until installed "
                "(see HANDOFF §6). Plumbing only."
            )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._last_origin = (p.x, p.y, p.z)

    def _on_cloud(self, msg: PointCloud2):
        if not _HAS_PC2_PY:
            return
        points = [
            (p[0], p[1], p[2])
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if not points:
            return

        # TODO(copilot): integrate into the TSDF.
        #   import numpy as np
        #   pts = np.asarray(points, dtype=np.float64)
        #   origin = np.asarray(self._last_origin, dtype=np.float64)
        #   self.vdb.integrate(pts, origin)   # carve from current sensor origin

        self._cloud_count += 1
        if self._cloud_count % self.mesh_every_n == 0:
            self._publish_mesh()

    def _publish_mesh(self):
        # TODO(copilot): extract + publish the live mesh.
        #   verts, tris = self.vdb.extract_triangle_mesh(min_weight=self.min_weight)
        #   marker.points = [Point(x=..., y=..., z=...) for each triangle vertex]
        if not _HAS_VDB:
            return
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scanner_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.85
        marker.color.a = 1.0
        marker.points = []  # TODO(copilot): fill from extracted triangles
        self.mesh_pub.publish(marker)

    def destroy_node(self):
        if self.get_parameter("save_on_shutdown").value and _HAS_VDB and self.vdb is not None:
            # TODO(copilot): write self.save_path PLY here.
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VdbMeshingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
