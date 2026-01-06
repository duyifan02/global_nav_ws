from __future__ import annotations

import os
import json
from typing import Any, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_interfaces.srv import SetVoxelMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover
    yaml = None


def _as_float3(values: Sequence[float]) -> Tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError('Expected 3 values')
    return float(values[0]), float(values[1]), float(values[2])


def _param_to_float3(value: Any) -> Tuple[float, float, float]:
    if isinstance(value, (list, tuple)):
        return _as_float3(value)
    if isinstance(value, str):
        if yaml is None:
            raise ValueError('YAML parsing unavailable; provide origin as a list')
        loaded = yaml.safe_load(value)
        return _as_float3(loaded)
    raise ValueError(f'Unsupported param type for float3: {type(value)}')


class NpyMapLoader(Node):
    def __init__(self) -> None:
        super().__init__('npy_map_loader')

        self.declare_parameter('npy_path', '')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('free_space_margin', 2.0)
        self.declare_parameter('origin', [0.0, 0.0, 0.0])
        self.declare_parameter('meta_path', '')
        self.declare_parameter('use_meta_resolution', True)
        self.declare_parameter('use_meta_origin', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('points_are_indices', False)
        self.declare_parameter('set_map_service', 'set_voxel_map')
        self.declare_parameter('wait_timeout_sec', 10.0)
        self.declare_parameter('call_set_map', True)

        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('cloud_topic', 'occupied_voxels')
        self.declare_parameter('max_viz_points', 2000000)
        self.declare_parameter('viz_stride', 1)

        self.declare_parameter('max_service_points', 5000000)
        self.declare_parameter('truncate_service_points', False)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._cloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('cloud_topic').get_parameter_value().string_value,
            qos,
        )

        self._client = self.create_client(
            SetVoxelMap,
            self.get_parameter('set_map_service').get_parameter_value().string_value,
        )

        self._run_once()

    def _run_once(self) -> None:
        npy_path = self.get_parameter('npy_path').get_parameter_value().string_value
        if not npy_path:
            self.get_logger().error('Parameter "npy_path" is empty')
            return
        if not os.path.exists(npy_path):
            self.get_logger().error(f'npy_path does not exist: {npy_path}')
            return

        # Load meta if provided (or auto-detect sidecar)
        resolution = float(self.get_parameter('resolution').value)
        free_space_margin = float(self.get_parameter('free_space_margin').value)
        origin = _param_to_float3(self.get_parameter('origin').value)

        meta_path = self.get_parameter('meta_path').get_parameter_value().string_value
        if not meta_path:
            base, ext = os.path.splitext(npy_path)
            candidate = base + '_meta.json'
            if os.path.exists(candidate):
                meta_path = candidate

        if meta_path:
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
                if bool(self.get_parameter('use_meta_resolution').value) and 'resolution' in meta:
                    resolution = float(meta['resolution'])
                if bool(self.get_parameter('use_meta_origin').value) and 'origin' in meta:
                    origin = _param_to_float3(meta['origin'])
                self.get_logger().info(f'Loaded meta: {meta_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load meta file {meta_path}: {e}')

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        raw = np.load(npy_path)
        arr = np.asarray(raw)
        if arr.ndim != 2 or arr.shape[1] != 3:
            self.get_logger().error(f'Expected npy shape (N,3), got {arr.shape}')
            return

        points_are_indices = bool(self.get_parameter('points_are_indices').value)
        if not points_are_indices:
            points_are_indices = np.issubdtype(arr.dtype, np.integer)

        if points_are_indices:
            idx = arr.astype(np.float32)
            pts = np.empty_like(idx, dtype=np.float32)
            pts[:, 0] = origin[0] + (idx[:, 0] + 0.5) * resolution
            pts[:, 1] = origin[1] + (idx[:, 1] + 0.5) * resolution
            pts[:, 2] = origin[2] + (idx[:, 2] + 0.5) * resolution
            self.get_logger().info(
                'Loaded indices npy; converted to world points with origin/resolution.'
            )
        else:
            pts = arr.astype(np.float32)
            self.get_logger().info('Loaded world-coordinate points npy.')

        num_points = int(pts.shape[0])
        self.get_logger().info(f'Loaded {num_points} occupied voxels')

        # Visualization publish (optionally subsample to avoid RViz overload)
        if bool(self.get_parameter('publish_cloud').value):
            max_viz = int(self.get_parameter('max_viz_points').value)
            stride = int(self.get_parameter('viz_stride').value)
            stride = max(1, stride)

            viz_pts = pts[::stride]
            if max_viz > 0 and viz_pts.shape[0] > max_viz:
                viz_pts = viz_pts[:max_viz]

            cloud = point_cloud2.create_cloud_xyz32(
                header=self._make_header(frame_id),
                points=viz_pts.tolist(),
            )
            self._cloud_pub.publish(cloud)
            self.get_logger().info(f'Published PointCloud2 with {viz_pts.shape[0]} points')

        # Service call to set voxel map
        if not bool(self.get_parameter('call_set_map').value):
            self.get_logger().info('call_set_map=false; skipping service call')
            return

        timeout = float(self.get_parameter('wait_timeout_sec').value)
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'Service {self._client.srv_name} not available after {timeout:.1f}s'
            )
            return

        max_srv = int(self.get_parameter('max_service_points').value)
        truncate = bool(self.get_parameter('truncate_service_points').value)
        srv_pts = pts
        if max_srv > 0 and srv_pts.shape[0] > max_srv:
            msg = (
                f'Occupied points = {srv_pts.shape[0]} exceeds max_service_points={max_srv}. '
            )
            if truncate:
                self.get_logger().warn(msg + 'Truncating for service call (map will be incomplete).')
                srv_pts = srv_pts[:max_srv]
            else:
                self.get_logger().error(msg + 'Set truncate_service_points=true or increase voxel size.')
                return

        req = SetVoxelMap.Request()
        req.resolution = float(resolution)
        req.free_space_margin = float(free_space_margin)
        req.occupied = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in srv_pts]

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error('set_voxel_map call failed or timed out')
            return

        resp = future.result()
        if resp.success:
            self.get_logger().info(f'set_voxel_map success: {resp.message}')
        else:
            self.get_logger().error(f'set_voxel_map failed: {resp.message}')

    def _make_header(self, frame_id: str):
        from std_msgs.msg import Header

        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h


def main() -> None:
    rclpy.init()
    node = NpyMapLoader()
    # Exit after one-shot execution
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
