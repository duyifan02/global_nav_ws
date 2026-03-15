#!/usr/bin/env python3
"""
Room Visualizer for topo_nav.

Calls GetRoomLabels service and publishes:
  - /topo_nav/room_cloud    : PointCloud2 colored by room id
  - /topo_nav/floor_cloud   : PointCloud2 colored by floor id
  - /topo_nav/persist_cloud : PointCloud2 colored by room birth_d (clearance)

Usage:
    ros2 run topo_nav room_visualizer
"""

import rclpy
from rclpy.node import Node

import struct
import math
import colorsys

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from nav_interfaces.srv import GetRoomLabels


# ── Colour helpers ────────────────────────────────────────────────────────────

def _room_colour(room_id: int, n_rooms: int) -> tuple:
    """Return (r, g, b) ∈ [0,255]³ for room_id using HSV colour wheel."""
    if n_rooms <= 1:
        return (100, 200, 255)
    hue = (room_id * 137.508) % 360.0  # golden-angle spacing
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 0.85, 0.90)
    return (int(r * 255), int(g * 255), int(b * 255))


def _floor_colour(floor_id: int) -> tuple:
    """Return a distinct colour for each floor."""
    palette = [
        (255,  80,  80),  # floor 0: red
        ( 80, 200,  80),  # floor 1: green
        ( 80, 130, 255),  # floor 2: blue
        (255, 200,  50),  # floor 3: yellow
        (200,  80, 255),  # floor 4: purple
    ]
    c = palette[floor_id % len(palette)]
    return c


def _persist_colour(val: float, vmin: float, vmax: float) -> tuple:
    """Blue (low clearance) → green → red (high clearance)."""
    if vmax <= vmin:
        return (128, 128, 128)
    t = (val - vmin) / (vmax - vmin)
    t = max(0.0, min(1.0, t))
    r, g, b = colorsys.hsv_to_rgb((1.0 - t) * 0.667, 1.0, 1.0)
    return (int(r * 255), int(g * 255), int(b * 255))


def _make_cloud(header: Header,
                points: list,
                colours: list) -> PointCloud2:
    """Build a PointCloud2 from list of (x,y,z) and list of (r,g,b)."""
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    data = bytearray()
    for (x, y, z), (r, g, b) in zip(points, colours):
        rgb_int = (r << 16) | (g << 8) | b
        data += struct.pack('ffff', x, y, z, struct.unpack('f', struct.pack('I', rgb_int))[0])

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width  = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = 16 * len(points)
    msg.data         = bytes(data)
    msg.is_dense     = True
    return msg


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class RoomVisualizerNode(Node):
    def __init__(self):
        super().__init__('room_visualizer')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)

        self._room_pub   = self.create_publisher(PointCloud2, '/topo_nav/room_cloud',    10)
        self._floor_pub  = self.create_publisher(PointCloud2, '/topo_nav/floor_cloud',   10)
        self._pers_pub   = self.create_publisher(PointCloud2, '/topo_nav/persist_cloud', 10)

        self._cli = self.create_client(GetRoomLabels, '/topo_nav/get_room_labels')

        rate = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self._cached_response = None
        self.get_logger().info('RoomVisualizer ready. Waiting for topo_nav node...')

    def _tick(self):
        if not self._cli.wait_for_service(timeout_sec=0.1):
            return
        if self._cached_response is None:
            # Request labels
            future = self._cli.call_async(GetRoomLabels.Request())
            future.add_done_callback(self._on_labels)
        else:
            self._publish(self._cached_response)

    def _on_labels(self, future):
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().warn('GetRoomLabels failed.')
            return
        self._cached_response = resp
        self._publish(resp)

    def _publish(self, resp):
        frame = self.get_parameter('frame_id').value
        header = Header()
        header.frame_id = frame
        header.stamp    = self.get_clock().now().to_msg()

        pts = [(p.x, p.y, p.z) for p in resp.surface_points]
        n = len(pts)
        if n == 0:
            return

        labels  = list(resp.room_labels)
        floors  = list(resp.floor_labels)
        pers    = list(resp.room_persistence)

        n_rooms = max(labels) + 1 if labels else 1

        # Room cloud
        room_colours = [_room_colour(lbl, n_rooms) if lbl >= 0 else (60, 60, 60)
                        for lbl in labels]
        self._room_pub.publish(_make_cloud(header, pts, room_colours))

        # Floor cloud
        floor_colours = [_floor_colour(f) if f >= 0 else (60, 60, 60)
                         for f in floors]
        self._floor_pub.publish(_make_cloud(header, pts, floor_colours))

        # Persistence cloud (use room birth_d as proxy for clearance)
        p_min = min(pers)
        p_max = max(pers)
        pers_colours = [_persist_colour(p, p_min, p_max) for p in pers]
        self._pers_pub.publish(_make_cloud(header, pts, pers_colours))


def main(args=None):
    rclpy.init(args=args)
    node = RoomVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
