# JPS Planner 3D - ROS 2 路径规划节点

基于 Surface 的 3D 路径规划算法，适用于地面机器人在复杂环境（楼梯、斜坡、台阶）中的导航。

## 快速开始

### 1. 编译

```bash
cd /home/steam/project/global_nav_ws && rm -rf build install log && source /opt/ros/jazzy/setup.bash && COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 2. 启动节点

```bash
# 使用默认参数
ros2 launch jps_planner_3d jps_planner.launch.py

# 使用自定义配置
ros2 launch jps_planner_3d jps_planner.launch.py config:=/path/to/your/config.yaml
```

### 3. 设置体素地图

调用 `/set_voxel_map` 服务设置占据体素地图：

```bash
ros2 service call /set_voxel_map nav_interfaces/srv/SetVoxelMap "{
  resolution: 0.2,
  occupied: [
    {x: 1.0, y: 0.0, z: 0.0},
    {x: 1.0, y: 0.5, z: 0.0},
    {x: 1.0, y: 1.0, z: 0.0}
  ]
}"
```

**参数说明**：
| 字段 | 类型 | 说明 |
|------|------|------|
| `resolution` | float64 | 体素分辨率（米），必须 > 0 |
| `occupied` | Point[] | 占据体素的世界坐标列表 |

### 4. 请求路径规划

调用 `/plan_path` 服务进行路径规划：

```bash
ros2 service call /plan_path nav_interfaces/srv/PlanPath "{
  start: {position: {x: 0.0, y: 0.0, z: 0.5}},
  goal: {position: {x: 5.0, y: 3.0, z: 0.5}},
  snap_to_surface: true
}"
```

**参数说明**：
| 字段 | 类型 | 说明 |
|------|------|------|
| `start` | Pose | 起点位姿（只使用 position） |
| `goal` | Pose | 终点位姿（只使用 position） |
| `snap_to_surface` | bool | 是否自动吸附到最近的有效 Surface |

**返回值**：
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 规划是否成功 |
| `message` | string | 状态信息或错误描述 |
| `path` | nav_msgs/Path | 规划的路径（仅当 success=true） |

---

## 服务接口

### SetVoxelMap.srv

```
float64 resolution
geometry_msgs/Point[] occupied
geometry_msgs/Point seed_point
---
bool success
string message
int64 surface_voxel_count
float64 traversability_time_ms
int64 map_memory_usage_bytes
```

### LoadPcdMap.srv

```
string pcd_path
float64 resolution
int32 min_points_per_voxel
float64 max_abs_coord
geometry_msgs/Point seed_point
---
bool success
string message
int64 raw_point_count
int64 occupied_voxel_count
int64 surface_voxel_count
float64 preprocessing_time_ms
float64 traversability_time_ms
int64 map_memory_usage_bytes
```

### PlanPath.srv

```
geometry_msgs/Pose start
geometry_msgs/Pose goal
bool snap_to_surface
---
bool success
string message
nav_msgs/Path path
float64 planning_time_ms
float64 traversability_time_ms
int64 expanded_nodes
int64 memory_usage_bytes
```

---

## 配置参数

配置文件位于 `config/jps_planner.yaml`：

```yaml
jps_planner_node:
  ros__parameters:
    # 坐标系
    frame_id: "map"

    # A* / JPS 规划参数
    heuristic_weight: 1.0      # 启发式权重
    max_z_change: 0.5          # 单步最大Z变化（米）
    z_penalty_weight: 3.0      # Z变化惩罚权重
    potential_weight: 100.0    # 贴墙惩罚权重

    # VoxelMap / Surface 参数
    connectivity_height_threshold: 0.6  # 可跨越台阶高度（米）
    surface_clearance_height: 0.8       # Surface上方空气高度（米）
    distance_field_max_distance: 5      # 距离场最大距离（网格单位）
```

---

## 错误信息

| 错误信息 | 原因 |
|----------|------|
| `Resolution must be > 0` | SetVoxelMap 参数无效 |
| `Voxel map is not ready` | 未调用 SetVoxelMap 或地图为空 |
| `Start or goal outside voxel map` | 起点/终点超出地图边界 |
| `Start is not on a valid surface` | 起点不在可行走表面上 |
| `Goal is not on a valid surface` | 终点不在可行走表面上 |
| `Planner returned empty path` | 无法找到有效路径 |

---

## 典型使用流程

```
1. 启动节点
   ros2 launch jps_planner_3d jps_planner.launch.py

2. 获取占据体素数据（来自 Octomap、点云等）

3. 调用 /set_voxel_map 设置地图
   - 每次地图更新都需要重新调用

4. 调用 /plan_path 请求路径
   - 起点/终点使用世界坐标
   - snap_to_surface=true 自动吸附到有效表面

5. 获取 nav_msgs/Path 结果
   - 可直接发送给轨迹跟踪控制器
```

---

## 文件结构

```
jps_planner_3d/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── jps_planner.yaml      # 参数配置
├── doc/
│   ├── README.md             # 本文档
│   └── algorithm.md          # 算法设计文档
├── include/
│   ├── jps_3d.hpp            # JPS规划器头文件
│   └── voxel_map.hpp         # 体素地图头文件
├── launch/
│   └── jps_planner.launch.py # 启动文件
├── src/
│   ├── jps_3d.cpp            # JPS规划器实现
│   ├── jps_planner_node.cpp  # ROS2节点
│   └── voxel_map.cpp         # 体素地图实现
└── srv/
    ├── PlanPath.srv          # 路径规划服务
    └── SetVoxelMap.srv       # 设置地图服务
```
