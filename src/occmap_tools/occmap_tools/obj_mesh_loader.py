from __future__ import annotations

import os
import json
from typing import Any, Sequence, Tuple, List, Set
import time

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


def load_obj_mesh(obj_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load OBJ file and return vertices and faces.
    
    Args:
        obj_path: Path to .obj file
        
    Returns:
        vertices: (N, 3) float array of vertex positions
        faces: (M, 3) int array of triangle vertex indices (0-based)
    """
    vertices = []
    faces = []
    
    with open(obj_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if not parts:
                continue
            
            if parts[0] == 'v':
                # Vertex: v x y z
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif parts[0] == 'f':
                # Face: f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
                # We only care about vertex indices (first number in each group)
                face_verts = []
                for i in range(1, len(parts)):
                    # Handle formats: "v", "v/vt", "v/vt/vn", "v//vn"
                    vert_idx = int(parts[i].split('/')[0])
                    # OBJ indices are 1-based, convert to 0-based
                    face_verts.append(vert_idx - 1)
                
                # Triangulate if necessary (assumes convex faces)
                for i in range(1, len(face_verts) - 1):
                    faces.append([face_verts[0], face_verts[i], face_verts[i + 1]])
    
    return np.array(vertices, dtype=np.float32), np.array(faces, dtype=np.int32)


def voxelize_triangle(
    v0: np.ndarray, 
    v1: np.ndarray, 
    v2: np.ndarray, 
    resolution: float, 
    origin: Tuple[float, float, float]
) -> Set[Tuple[int, int, int]]:
    """
    Voxelize a single triangle using surface sampling.
    
    Args:
        v0, v1, v2: Triangle vertices (3D points)
        resolution: Voxel size in meters
        origin: World origin (ox, oy, oz)
        
    Returns:
        Set of voxel indices (ix, iy, iz) that the triangle intersects
    """
    voxels = set()
    
    # Compute triangle bounding box in voxel space
    vertices = np.array([v0, v1, v2])
    min_bound = vertices.min(axis=0)
    max_bound = vertices.max(axis=0)
    
    # Convert to voxel indices with padding
    ox, oy, oz = origin
    min_idx = np.floor((min_bound - np.array([ox, oy, oz])) / resolution).astype(int) - 1
    max_idx = np.ceil((max_bound - np.array([ox, oy, oz])) / resolution).astype(int) + 1
    
    # Sample triangle surface
    # Use barycentric coordinates to sample uniformly
    num_samples = max(10, int(np.linalg.norm(v1 - v0) / resolution * 2),
                          int(np.linalg.norm(v2 - v0) / resolution * 2))
    
    for i in range(num_samples):
        # Random barycentric coordinates
        r1 = np.random.random()
        r2 = np.random.random()
        
        # Ensure point is inside triangle
        if r1 + r2 > 1:
            r1 = 1 - r1
            r2 = 1 - r2
        
        # Barycentric interpolation
        point = v0 + r1 * (v1 - v0) + r2 * (v2 - v0)
        
        # Convert to voxel index
        ix = int(np.floor((point[0] - ox) / resolution))
        iy = int(np.floor((point[1] - oy) / resolution))
        iz = int(np.floor((point[2] - oz) / resolution))
        
        voxels.add((ix, iy, iz))
    
    # Also sample triangle edges to ensure connectivity
    for edge in [(v0, v1), (v1, v2), (v2, v0)]:
        edge_length = np.linalg.norm(edge[1] - edge[0])
        num_edge_samples = max(2, int(edge_length / resolution * 2))
        
        for i in range(num_edge_samples):
            t = i / (num_edge_samples - 1) if num_edge_samples > 1 else 0.5
            point = edge[0] + t * (edge[1] - edge[0])
            
            ix = int(np.floor((point[0] - ox) / resolution))
            iy = int(np.floor((point[1] - oy) / resolution))
            iz = int(np.floor((point[2] - oz) / resolution))
            
            voxels.add((ix, iy, iz))
    
    return voxels


def voxelize_mesh(
    vertices: np.ndarray, 
    faces: np.ndarray, 
    resolution: float,
    origin: Tuple[float, float, float],
    logger=None
) -> np.ndarray:
    """
    Convert mesh to voxel occupancy grid using surface voxelization.
    
    Args:
        vertices: (N, 3) vertex positions
        faces: (M, 3) triangle indices
        resolution: Voxel size in meters
        origin: World origin (ox, oy, oz)
        logger: Optional ROS logger
        
    Returns:
        (K, 3) array of occupied voxel world coordinates
    """
    if logger:
        logger.info(f'Voxelizing mesh: {len(vertices)} vertices, {len(faces)} faces, resolution={resolution}m')
    
    start_time = time.time()
    
    # Accumulate all occupied voxels
    all_voxels = set()
    
    for i, face in enumerate(faces):
        if logger and i > 0 and i % 10000 == 0:
            logger.info(f'  Processed {i}/{len(faces)} faces ({100*i/len(faces):.1f}%)')
        
        v0 = vertices[face[0]]
        v1 = vertices[face[1]]
        v2 = vertices[face[2]]
        
        triangle_voxels = voxelize_triangle(v0, v1, v2, resolution, origin)
        all_voxels.update(triangle_voxels)
    
    # Convert voxel indices to world coordinates
    ox, oy, oz = origin
    occupied_points = np.array([
        [ox + (ix + 0.5) * resolution,
         oy + (iy + 0.5) * resolution,
         oz + (iz + 0.5) * resolution]
        for ix, iy, iz in all_voxels
    ], dtype=np.float32)
    
    elapsed = time.time() - start_time
    if logger:
        logger.info(f'Voxelization complete: {len(occupied_points)} occupied voxels in {elapsed:.2f}s')
    
    return occupied_points


class ObjMeshLoader(Node):
    def __init__(self) -> None:
        super().__init__('obj_mesh_loader')

        # File paths
        self.declare_parameter('obj_path', '')
        self.declare_parameter('meta_path', '')
        
        # Voxelization parameters
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin', [0.0, 0.0, 0.0])
        self.declare_parameter('auto_center', False)  # Auto-center mesh at origin
        
        # Meta override
        self.declare_parameter('use_meta_resolution', True)
        self.declare_parameter('use_meta_origin', True)
        
        # ROS parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('set_map_service', 'set_voxel_map')
        self.declare_parameter('wait_timeout_sec', 10.0)
        self.declare_parameter('call_set_map', True)

        # Visualization
        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('cloud_topic', 'occupied_voxels')
        self.declare_parameter('max_viz_points', 2000000)
        self.declare_parameter('viz_stride', 1)

        # Service limits
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
        obj_path = self.get_parameter('obj_path').get_parameter_value().string_value
        if not obj_path:
            self.get_logger().error('Parameter "obj_path" is empty')
            return
        if not os.path.exists(obj_path):
            self.get_logger().error(f'obj_path does not exist: {obj_path}')
            return

        # Load parameters
        resolution = float(self.get_parameter('resolution').value)
        origin = _param_to_float3(self.get_parameter('origin').value)
        auto_center = bool(self.get_parameter('auto_center').value)

        # Load meta if provided (or auto-detect sidecar)
        meta_path = self.get_parameter('meta_path').get_parameter_value().string_value
        if not meta_path:
            base, ext = os.path.splitext(obj_path)
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

        # Load OBJ mesh
        self.get_logger().info(f'Loading OBJ mesh from: {obj_path}')
        try:
            vertices, faces = load_obj_mesh(obj_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load OBJ file: {e}')
            return

        if len(vertices) == 0 or len(faces) == 0:
            self.get_logger().error('OBJ file contains no geometry')
            return

        self.get_logger().info(f'Loaded mesh: {len(vertices)} vertices, {len(faces)} faces')

        # Auto-center if requested
        if auto_center:
            mesh_center = vertices.mean(axis=0)
            origin = tuple(-mesh_center)
            self.get_logger().info(f'Auto-centered mesh, new origin: {origin}')

        # Voxelize mesh
        pts = voxelize_mesh(vertices, faces, resolution, origin, self.get_logger())
        
        if len(pts) == 0:
            self.get_logger().error('Voxelization produced no occupied voxels')
            return

        num_points = int(pts.shape[0])
        self.get_logger().info(f'Generated {num_points} occupied voxels')

        # Visualization publish
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
                self.get_logger().error(msg + 'Set truncate_service_points=true or increase resolution.')
                return

        req = SetVoxelMap.Request()
        req.resolution = float(resolution)
        req.occupied = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in srv_pts]

        self.get_logger().info('Calling set_voxel_map service...')
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
    node = ObjMeshLoader()
    # Exit after one-shot execution
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
