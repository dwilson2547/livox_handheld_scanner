#!/usr/bin/env python3
"""
vdb_meshing_node
================
Wraps VDBFusion to incrementally integrate the LIO-registered point cloud into
a TSDF volume and publish a live triangle mesh for the operator viewer.

STATUS: functional when VDBFusion is installed. The node integrates registered
clouds into a TSDF, periodically republishes the extracted surface as a
TRIANGLE_LIST marker, and writes a PLY on shutdown. If VDBFusion is missing,
the node stays up and logs that meshing is disabled so the rest of the pipeline
can still run.

Why VDBFusion: sparse-volume TSDF, fast incremental integration, gives a real
surface (not just points) so the operator can judge coverage holes, and the
same volume can be re-integrated offline at finer voxel size for the print mesh.
"""
from pathlib import Path

import numpy as np
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

try:
    from vdbfusion import VDBVolume

    _HAS_VDB = True
except Exception:
    VDBVolume = None
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
        self.declare_parameter("publish_mesh", True)
        self.declare_parameter("save_on_shutdown", True)
        self.declare_parameter("save_path", "sessions/mesh_live.ply")

        self.map_frame = self.get_parameter("map_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.sdf_trunc = float(self.get_parameter("sdf_trunc").value)
        self.space_carving = bool(self.get_parameter("space_carving").value)
        self.mesh_every_n = int(self.get_parameter("mesh_every_n_clouds").value)
        self.min_weight = float(self.get_parameter("min_weight").value)
        self.publish_mesh = bool(self.get_parameter("publish_mesh").value)
        self.save_path = Path(self.get_parameter("save_path").value).expanduser()

        self._last_origin = (0.0, 0.0, 0.0)  # sensor origin for TSDF carving
        self._cloud_count = 0

        self.vdb = None
        if _HAS_VDB:
            self.vdb = VDBVolume(
                self.voxel_size,
                self.sdf_trunc,
                self.space_carving,
            )

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
        if not _HAS_PC2_PY or not _HAS_VDB or self.vdb is None:
            return
        point_rows = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not point_rows:
            return

        first_row = point_rows[0]
        if isinstance(first_row, np.void) and first_row.dtype.names:
            points = np.asarray(
                [(row["x"], row["y"], row["z"]) for row in point_rows],
                dtype=np.float64,
            )
        else:
            points = np.asarray(point_rows, dtype=np.float64)
        if points.size == 0:
            return

        origin = np.asarray(self._last_origin, dtype=np.float64)
        self.vdb.integrate(points=points, extrinsic=origin)

        self._cloud_count += 1
        if (
            self.publish_mesh
            and self.mesh_every_n > 0
            and self._cloud_count % self.mesh_every_n == 0
        ):
            self._publish_mesh()

    def _publish_mesh(self):
        if not _HAS_VDB or self.vdb is None:
            return
        verts, tris = self.vdb.extract_triangle_mesh(min_weight=self.min_weight)
        if len(verts) == 0 or len(tris) == 0:
            self._publish_empty_mesh()
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scanner_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.85
        marker.color.a = 1.0
        marker.points = []
        for triangle in tris:
            for vertex_idx in triangle:
                vx, vy, vz = verts[int(vertex_idx)]
                marker.points.append(Point(x=float(vx), y=float(vy), z=float(vz)))
        self.mesh_pub.publish(marker)

    def _publish_empty_mesh(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scanner_mesh"
        marker.id = 0
        marker.action = Marker.DELETE
        self.mesh_pub.publish(marker)

    @staticmethod
    def _write_ascii_ply(path: Path, vertices: np.ndarray, triangles: np.ndarray):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            handle.write("ply\n")
            handle.write("format ascii 1.0\n")
            handle.write(f"element vertex {len(vertices)}\n")
            handle.write("property float x\n")
            handle.write("property float y\n")
            handle.write("property float z\n")
            handle.write(f"element face {len(triangles)}\n")
            handle.write("property list uchar int vertex_indices\n")
            handle.write("end_header\n")
            for vertex in vertices:
                handle.write(
                    f"{float(vertex[0])} {float(vertex[1])} {float(vertex[2])}\n"
                )
            for triangle in triangles:
                handle.write(
                    f"3 {int(triangle[0])} {int(triangle[1])} {int(triangle[2])}\n"
                )

    def destroy_node(self):
        if (
            self.get_parameter("save_on_shutdown").value
            and _HAS_VDB
            and self.vdb is not None
        ):
            verts, tris = self.vdb.extract_triangle_mesh(min_weight=self.min_weight)
            if len(verts) > 0 and len(tris) > 0:
                output_path = self.save_path
                if not output_path.is_absolute():
                    output_path = Path.cwd() / output_path
                self._write_ascii_ply(output_path, verts, tris)
                self.get_logger().info(f"wrote live mesh to {output_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VdbMeshingNode()
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
