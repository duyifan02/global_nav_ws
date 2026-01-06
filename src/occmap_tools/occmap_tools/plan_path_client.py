from __future__ import annotations

from typing import Any, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose
from nav_interfaces.srv import PlanPath
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

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
            raise ValueError('YAML parsing unavailable; provide start/goal as a list')
        loaded = yaml.safe_load(value)
        return _as_float3(loaded)
    raise ValueError(f'Unsupported param type for float3: {type(value)}')


class PlanPathClient(Node):
    def __init__(self) -> None:
        super().__init__('plan_path_client')

        self.declare_parameter('start', [0.0, 0.0, 0.0])
        self.declare_parameter('goal', [1.0, 0.0, 0.0])
        self.declare_parameter('snap_to_surface', True)
        self.declare_parameter('plan_service', 'plan_path')
        self.declare_parameter('wait_timeout_sec', 10.0)
        self.declare_parameter('publish_path', True)
        self.declare_parameter('path_topic', 'planned_path')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._path_pub = self.create_publisher(
            Path,
            self.get_parameter('path_topic').get_parameter_value().string_value,
            qos,
        )

        self._client = self.create_client(
            PlanPath,
            self.get_parameter('plan_service').get_parameter_value().string_value,
        )

        self._run_once()

    def _run_once(self) -> None:
        start = _param_to_float3(self.get_parameter('start').value)
        goal = _param_to_float3(self.get_parameter('goal').value)
        snap = bool(self.get_parameter('snap_to_surface').value)
        timeout = float(self.get_parameter('wait_timeout_sec').value)

        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'Service {self._client.srv_name} not available after {timeout:.1f}s'
            )
            return

        req = PlanPath.Request()
        req.start = Pose()
        req.start.position.x = start[0]
        req.start.position.y = start[1]
        req.start.position.z = start[2]
        req.start.orientation.w = 1.0

        req.goal = Pose()
        req.goal.position.x = goal[0]
        req.goal.position.y = goal[1]
        req.goal.position.z = goal[2]
        req.goal.orientation.w = 1.0

        req.snap_to_surface = snap

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error('plan_path call failed or timed out')
            return

        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'plan_path failed: {resp.message}')
            return

        self.get_logger().info(f'plan_path success: {resp.message}, poses={len(resp.path.poses)}')
        if bool(self.get_parameter('publish_path').value):
            self._path_pub.publish(resp.path)


def main() -> None:
    rclpy.init()
    node = PlanPathClient()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
