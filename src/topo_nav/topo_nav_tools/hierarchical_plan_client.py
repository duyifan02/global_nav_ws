#!/usr/bin/env python3
"""
Hierarchical planning client for topo_nav.

Usage:
    ros2 run topo_nav hierarchical_plan_client \
        --ros-args -p start_x:=1.0 -p start_y:=2.0 -p start_z:=0.1 \
                   -p goal_x:=10.0 -p goal_y:=5.0  -p goal_z:=0.1
"""

import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_interfaces.srv import PlanHierarchical, BuildTopoMap


class HierarchicalPlanClient(Node):
    def __init__(self):
        super().__init__('hierarchical_plan_client')

        # Pose parameters
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('goal_x',  5.0)
        self.declare_parameter('goal_y',  5.0)
        self.declare_parameter('goal_z',  0.0)
        self.declare_parameter('robot_diameter_m', 0.4)
        self.declare_parameter('snap_to_surface', True)
        # If True, call BuildTopoMap first (useful for one-shot testing)
        self.declare_parameter('build_first', False)
        self.declare_parameter('persistence_tau', 3.0)

        self._plan_cli  = self.create_client(PlanHierarchical, 'topo_nav/plan_hierarchical')
        self._build_cli = self.create_client(BuildTopoMap,    'topo_nav/build_topo_map')

    def run(self):
        # Optionally build topo map first
        if self.get_parameter('build_first').value:
            self.get_logger().info('Calling BuildTopoMap...')
            if not self._build_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('build_topo_map service not available.')
                return

            req = BuildTopoMap.Request()
            req.persistence_tau       = self.get_parameter('persistence_tau').value
            req.floor_separation_m    = 2.5
            req.corridor_aspect_ratio = 3.0
            req.staircase_variance_m  = 0.5
            req.robot_diameter_m      = self.get_parameter('robot_diameter_m').value

            future = self._build_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            if resp is None or not resp.success:
                self.get_logger().error(f'BuildTopoMap failed: {resp.message if resp else "None"}')
                return
            self.get_logger().info(
                f'BuildTopoMap: {resp.num_rooms} rooms, {resp.num_floors} floors, '
                f'{resp.build_time_ms:.1f} ms')

        # Plan
        if not self._plan_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('plan_hierarchical service not available.')
            return

        req = PlanHierarchical.Request()
        req.start.position.x = self.get_parameter('start_x').value
        req.start.position.y = self.get_parameter('start_y').value
        req.start.position.z = self.get_parameter('start_z').value
        req.start.orientation.w = 1.0
        req.goal.position.x  = self.get_parameter('goal_x').value
        req.goal.position.y  = self.get_parameter('goal_y').value
        req.goal.position.z  = self.get_parameter('goal_z').value
        req.goal.orientation.w = 1.0
        req.snap_to_surface  = self.get_parameter('snap_to_surface').value
        req.robot_diameter_m = self.get_parameter('robot_diameter_m').value

        self.get_logger().info(
            f'Planning from ({req.start.position.x:.2f}, '
            f'{req.start.position.y:.2f}, {req.start.position.z:.2f}) '
            f'to ({req.goal.position.x:.2f}, {req.goal.position.y:.2f}, '
            f'{req.goal.position.z:.2f})...')

        future = self._plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None:
            self.get_logger().error('Service call failed (None response).')
            return

        if not resp.success:
            self.get_logger().error(f'Planning failed: {resp.message}')
            return

        self.get_logger().info(
            f'\n── Hierarchical Plan ──────────────────────────────\n'
            f'  Status       : {resp.message}\n'
            f'  Path length  : {resp.path_length_m:.3f} m\n'
            f'  Waypoints    : {len(resp.path.poses)}\n'
            f'  Room sequence: {list(resp.room_sequence)}\n'
            f'  Floor seq    : {list(resp.floor_sequence)}\n'
            f'  Time total   : {resp.planning_time_ms:.3f} ms\n'
            f'    Floor step : {resp.floor_planning_ms:.3f} ms\n'
            f'    Room step  : {resp.room_planning_ms:.3f} ms\n'
            f'    Voxel step : {resp.voxel_planning_ms:.3f} ms\n'
            f'  Voxel nodes  : {resp.voxel_nodes_expanded}\n'
            f'──────────────────────────────────────────────────')


def main(args=None):
    rclpy.init(args=args)
    node = HierarchicalPlanClient()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
