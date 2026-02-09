#!/usr/bin/env python3
"""
Distance Field Visualizer Node

Requests distance field data from JPS planner and publishes as colored PointCloud2.
Color mapping: Blue (far from obstacles) -> Red (near obstacles)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_interfaces.srv import GetDistanceField
import struct
import numpy as np


class DistanceFieldVisualizer(Node):
    def __init__(self):
        super().__init__('distance_field_visualizer')
        
        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('service_timeout', 5.0)  # seconds
        
        self.update_rate = self.get_parameter('update_rate').value
        self.service_timeout = self.get_parameter('service_timeout').value
        
        # Service client
        self.client = self.create_client(GetDistanceField, '/get_distance_field')
        
        # Publisher
        self.pub = self.create_publisher(PointCloud2, '/distance_field_cloud', 10)
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        self.get_logger().info(f'Distance field visualizer started (update rate: {self.update_rate} Hz)')
        
    def timer_callback(self):
        """Periodically request and publish distance field visualization"""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /get_distance_field not available', throttle_duration_sec=5.0)
            return
        
        # Request distance field
        request = GetDistanceField.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)
        
    def service_response_callback(self, future):
        """Handle service response and publish PointCloud2"""
        try:
            response = future.result()
            
            if not response.success:
                self.get_logger().error(f'Failed to get distance field: {response.message}')
                return
            
            if len(response.surface_points) == 0:
                self.get_logger().warn('No surface points in distance field', throttle_duration_sec=5.0)
                return
            
            # Convert to PointCloud2
            cloud_msg = self.create_colored_pointcloud(
                response.surface_points,
                response.distances,
                response.resolution
            )
            
            self.pub.publish(cloud_msg)
            self.get_logger().info(
                f'Published distance field: {len(response.surface_points)} points',
                throttle_duration_sec=2.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def create_colored_pointcloud(self, points, distances, resolution):
        """
        Create PointCloud2 with RGB colors based on distance values.
        
        Color mapping:
        - Distance 0 (obstacle-adjacent): Red (255, 0, 0)
        - Max distance: Blue (0, 0, 255)
        - Intermediate: Linear interpolation
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        
        # PointCloud2 fields: x, y, z, rgb
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        # Normalize distances to [0, 1]
        distances_array = np.array(distances, dtype=np.float32)
        max_dist = distances_array.max() if len(distances_array) > 0 else 1.0
        normalized = distances_array / max_dist if max_dist > 0 else distances_array
        
        # Build point cloud data
        cloud_data = []
        for pt, norm_dist in zip(points, normalized):
            # Color mapping: blue (far) -> red (near)
            # Blue component: high when far from obstacles
            # Red component: high when near obstacles
            r = int((1.0 - norm_dist) * 255)
            g = 0
            b = int(norm_dist * 255)
            
            # Pack RGB into uint32 (format: 0x00RRGGBB)
            rgb_packed = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            
            cloud_data.append(struct.pack('fffI', pt.x, pt.y, pt.z, rgb_packed))
        
        # Concatenate all points
        data = b''.join(cloud_data)
        
        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=16,  # 4 floats * 4 bytes (x,y,z) + 1 uint32 (rgb)
            row_step=16 * len(points),
            data=data
        )
        
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    node = DistanceFieldVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
