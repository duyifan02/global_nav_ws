# 3D Planner Benchmark 使用指南

## 概述

本工具用于对比测试 JPS 3D、PCT、3D2M 等路径规划算法的性能。

**核心指标（纯算法性能，不含ROS通信开销）**:
- 规划时间 (planning_time_ms)
- 扩展节点数 (expanded_nodes)
- 路径长度 (path_length_m)
- 成功率 (success_rate)

---

## 快速开始：RViz2 交互式 Benchmark（推荐）

使用 RViz2 手动选择起点/终点，精确控制测试条件。

### 1. 启动 JPS 规划节点

```bash
# 终端 1
source /opt/ros/jazzy/setup.bash
source ~/project/planner_benchmark/global_nav_ws/install/setup.bash
ros2 run jps_planner_3d jps_planner_node
```

### 2. 启动 RViz2

```bash
# 终端 2
source /opt/ros/jazzy/setup.bash
source ~/project/planner_benchmark/global_nav_ws/install/setup.bash
rviz2
```

RViz2 配置：
1. **Fixed Frame**: `map`
2. **Add** → PointCloud2 → Topic: `/benchmark/pointcloud`
3. **Add** → Path → Topic: `/benchmark/path`
4. **Add** → MarkerArray → Topic: `/benchmark/markers`
5. 工具栏选择 **Publish Point** 工具

### 3. 运行 Benchmark

```bash
# 终端 3
source /opt/ros/jazzy/setup.bash
source ~/project/planner_benchmark/global_nav_ws/install/setup.bash
cd ~/project/planner_benchmark/benchmark/tools
/usr/bin/python3.12  rviz_benchmark.py --scene building --trials 1
```

### 4. 操作流程

1. 点云会自动显示在 RViz2 中
2. 用 **Publish Point** 工具点击选择 **起点**（绿色球标记）
3. 再点击选择 **终点**（红色球标记）
4. 自动运行 N 次规划测试，实时显示结果
5. 继续点击开始新一轮测试
6. **Ctrl+C** 退出并保存所有结果

### 命令参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--scene` | 场景名称 (spiral, building, plaza) | spiral |
| `--resolution` | 体素分辨率 (m) | 0.1 |
| `--trials` | 每组起终点的测试次数 | 100 |

### 输出示例

```
运行 100 次规划测试
起点: [-20.5, -10.2, 5.0]
终点: [-35.2, -15.8, 12.0]
直线距离: 18.52m
============================================================
  [  1/100] ✓ 2.34ms | 25.67m | 1523 nodes
  [  2/100] ✓ 2.28ms | 25.67m | 1523 nodes
  ...

本轮统计 (纯算法耗时，不含ROS通信):
  成功率: 100.0%
  规划时间: 2.31 ± 0.12 ms
            min=2.15, max=2.89 ms
  路径长度: 25.67 ± 0.00 m
  扩展节点: 1523 ± 0
```

---

## 自动化 Benchmark（批量测试）

适用于预定义起终点的批量测试。

```bash
cd benchmark
python run_benchmark.py --planners JPS --scenes spiral --trials 50
```

### 命令参数

| 参数 | 选项 | 默认值 |
|------|------|--------|
| `--planners` | PCT, JPS, 3D2M, all | PCT, JPS |
| `--scenes` | spiral, building, plaza, all | spiral |
| `--trials` | 任意整数 | 50 |
| `--resolution` | 体素分辨率(m) | 0.1 |
| `--output` | 结果目录 | ./results |

---

## 各规划器运行前提

### JPS 3D (ROS2 Jazzy)

```bash
# 编译（首次或修改后）
source /opt/ros/jazzy/setup.bash
cd ~/project/planner_benchmark/global_nav_ws
colcon build --symlink-install

# 运行节点
source install/setup.bash
ros2 run jps_planner_3d jps_planner_node
```

**服务接口**:
- `/set_voxel_map` (SetVoxelMap) - 设置体素地图
- `/plan_path` (PlanPath) - 规划路径，返回纯算法耗时

### PCT

需要预生成 tomogram 文件：
```bash
# 检查文件是否存在
ls PCT_planner/rsc/tomogram/{scene}.pickle

# 如不存在，生成
cd PCT_planner/tomography/scripts
python tomography.py --scene Spiral
```

### 3D2M (ROS1)

需单独运行，不支持 Python 直接调用：
```bash
# 1. 启用 benchmark 模式
vim 3D2M-planner/planner/src/planner_manager/src/planner_manager.cpp
# 取消注释: #define BENCHMARKTIME

# 2. 编译运行
cd 3D2M-planner/planner && catkin_make
source devel/setup.bash
roslaunch planner_manager scene1.launch
```

---

## 输出文件

### RViz2 Benchmark 结果

保存在 `benchmark/results/` 目录：
```
jps_benchmark_{scene}_{timestamp}.json
```

JSON 结构：
```json
{
  "planner": "JPS_3D",
  "timestamp": "20260210_123456",
  "map_info": {
    "scene": "spiral",
    "resolution": 0.1,
    "num_voxels": 231884,
    "bounds_min": [-61.4, -32.2, -0.6],
    "bounds_max": [20.2, 8.6, 22.8]
  },
  "cumulative_stats": {
    "total_runs": 5,
    "total_trials": 500,
    "success_rate": 0.98,
    "planning_time_ms": {
      "mean": 2.34,
      "std": 0.45,
      "min": 1.82,
      "max": 5.67,
      "median": 2.21
    },
    "path_length_m": {...},
    "expanded_nodes": {...}
  },
  "runs": [
    {
      "start": [-20.5, -10.2, 5.0],
      "goal": [-35.2, -15.8, 12.0],
      "euclidean_distance": 18.52,
      "stats": {...},
      "trials": [...]
    }
  ]
}
```

### 自动化 Benchmark 结果

- `benchmark_YYYYMMDD_HHMMSS.json` — 原始数据
- `{planner}_{scene}_*.csv` — 分项 CSV

---

## 文件结构

```
benchmark/
├── USAGE.md                  # 本文档
├── run_benchmark.py          # 自动化benchmark主入口
├── config.py                 # 场景/参数配置
├── metrics.py                # 指标计算
├── converters/
│   └── pcd_to_voxel.py       # PCD→体素转换（支持ASCII/Binary）
├── planners/
│   ├── pct_wrapper.py        # PCT封装
│   ├── jps_wrapper.py        # JPS封装 (ROS2)
│   └── d2m_wrapper.py        # 3D2M封装 (ROS1)
├── tools/
│   └── rviz_benchmark.py     # RViz2交互式benchmark工具
└── results/                  # 输出结果目录
```

---

## 注意事项

1. **环境要求**: ROS2 Jazzy + Python 3.12（系统Python，不要用conda）
2. **性能指标**: `planning_time_ms` 是C++端测量的纯算法耗时，不含ROS服务调用开销
3. **选点技巧**: 选择点云表面附近的点，`snap_to_surface=True` 会自动吸附到最近体素
4. **多次测试**: 同一起终点多次测试用于统计稳定性（JPS是确定性算法，结果应一致）
