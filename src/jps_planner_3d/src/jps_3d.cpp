#include "jps_planner_3d/jps_3d.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include <queue>

namespace jps_planner_3d
{

// 用1D索引代替tuple，大幅提升性能
using NodeIdx = size_t;

JPS3D::JPS3D(
    std::shared_ptr<VoxelMap> voxel_map,
    double heuristic_weight,
    double max_z_change,
    double z_penalty_weight,
    double potential_weight)
: voxel_map_(voxel_map)
, heuristic_weight_(heuristic_weight)
, max_z_change_(max_z_change)
, z_penalty_weight_(z_penalty_weight)
, potential_weight_(potential_weight)
, max_iterations_(50000)
{
}

JPS3D::~JPS3D()
{
}

// 将3D坐标编码为1D索引
inline NodeIdx encode(int x, int y, int z, int ny, int nz) {
    return static_cast<NodeIdx>(x) * ny * nz + static_cast<NodeIdx>(y) * nz + z;
}

// 解码1D索引为3D坐标
inline std::tuple<int, int, int> decode(NodeIdx idx, int ny, int nz) {
    int x = static_cast<int>(idx / (ny * nz));
    int rem = static_cast<int>(idx % (ny * nz));
    int y = rem / nz;
    int z = rem % nz;
    return {x, y, z};
}

std::vector<std::tuple<double, double, double>> JPS3D::plan(
	const std::tuple<double, double, double>& start_pos,
	const std::tuple<double, double, double>& goal_pos,
	bool snap_to_surface)
{
    search_stats_.clear();

    auto start_idx = voxel_map_->worldToGrid(std::get<0>(start_pos),
                                              std::get<1>(start_pos),
                                              std::get<2>(start_pos));
    auto goal_idx = voxel_map_->worldToGrid(std::get<0>(goal_pos),
                                             std::get<1>(goal_pos),
                                             std::get<2>(goal_pos));

	if (!voxel_map_->isValidIdx(std::get<0>(start_idx), std::get<1>(start_idx), std::get<2>(start_idx)) ||
		!voxel_map_->isValidIdx(std::get<0>(goal_idx), std::get<1>(goal_idx), std::get<2>(goal_idx))) {
		search_stats_["failure_reason"] = 1;
		return {};
	}

	auto [sx, sy, sz] = start_idx;
	if (!voxel_map_->isSurfaceValid(sx, sy, sz)) {
		if (!snap_to_surface) {
			search_stats_["failure_reason"] = 2;
			return {};
		}

		int search_radius = static_cast<int>(5.0 / voxel_map_->getResolution());
		auto nearest = voxel_map_->findNearestValidSurface(sx, sy, sz, search_radius);

		if (std::get<0>(nearest) >= 0) {
			start_idx = nearest;
		} else {
			search_stats_["failure_reason"] = 2;
			return {};
		}
	}

	auto [gx, gy, gz] = goal_idx;
	if (!voxel_map_->isSurfaceValid(gx, gy, gz)) {
		if (!snap_to_surface) {
			search_stats_["failure_reason"] = 3;
			return {};
		}

		int search_radius = static_cast<int>(5.0 / voxel_map_->getResolution());
		auto nearest = voxel_map_->findNearestValidSurface(gx, gy, gz, search_radius);

		if (std::get<0>(nearest) >= 0) {
			goal_idx = nearest;
		} else {
			search_stats_["failure_reason"] = 3;
			return {};
		}
	}

    auto path_indices = jpsSearch(start_idx, goal_idx);

    if (path_indices.empty()) {
        return {};
    }

    std::vector<std::tuple<double, double, double>> path;
    for (const auto& idx : path_indices) {
        auto [ix, iy, iz] = idx;
        auto world_pos = voxel_map_->gridToWorld(ix, iy, iz);
        path.push_back(world_pos);
    }

    return path;
}

std::vector<std::tuple<int, int, int>> JPS3D::jpsSearch(
    const std::tuple<int, int, int>& start,
    const std::tuple<int, int, int>& goal)
{
    auto [nx, ny, nz] = voxel_map_->getGridSize();
    const NodeIdx total_nodes = static_cast<NodeIdx>(nx) * ny * nz;
    
    // 扁平化数组优化 - O(1) 访问，缓存友好
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(total_nodes, INF);
    std::vector<double> f_score(total_nodes, INF);
    std::vector<NodeIdx> came_from(total_nodes, std::numeric_limits<NodeIdx>::max());
    std::vector<bool> closed(total_nodes, false);
    
    // 编码起点和终点
    auto [sx, sy, sz] = start;
    auto [gx, gy, gz] = goal;
    NodeIdx start_idx = encode(sx, sy, sz, ny, nz);
    NodeIdx goal_idx = encode(gx, gy, gz, ny, nz);
    
    // 优先队列: (f_score, g_score, node_idx)
    using QueueElement = std::tuple<double, double, NodeIdx>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> open_set;
    
    g_score[start_idx] = 0.0;
    f_score[start_idx] = heuristic(start, goal);
    open_set.push({f_score[start_idx], 0.0, start_idx});
    
    int iterations = 0;
    int expanded = 0;
    
    while (!open_set.empty() && iterations < max_iterations_) {
        iterations++;
        auto [current_f, current_g, current_idx] = open_set.top();
        open_set.pop();
        
        if (closed[current_idx]) continue;
        
        auto [cx, cy, cz] = decode(current_idx, ny, nz);
        
        // 到达目标
        if (current_idx == goal_idx) {
            search_stats_["expanded_nodes"] = expanded;
            search_stats_["iterations"] = iterations;
            search_stats_["open_set_size"] = static_cast<int>(open_set.size());
            
            // 重建路径
            std::vector<std::tuple<int, int, int>> path;
            NodeIdx node = goal_idx;
            while (node != std::numeric_limits<NodeIdx>::max()) {
                auto [x, y, z] = decode(node, ny, nz);
                path.push_back({x, y, z});
                if (node == start_idx) break;
                node = came_from[node];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        closed[current_idx] = true;
        expanded++;
        
        // 获取邻居
        auto current_3d = std::make_tuple(cx, cy, cz);
        auto parent_3d = (came_from[current_idx] != std::numeric_limits<NodeIdx>::max()) 
            ? decode(came_from[current_idx], ny, nz) 
            : std::make_tuple(-1, -1, -1);
        auto successors = getSuccessors(current_3d, parent_3d, goal);
        
        for (const auto& neighbor : successors) {
            auto [nx_, ny_, nz_] = neighbor;
            NodeIdx neighbor_idx = encode(nx_, ny_, nz_, ny, nz);
            
            if (closed[neighbor_idx]) continue;
            
            double dist = distance(current_3d, neighbor);
            double tentative_g = g_score[current_idx] + dist;
            
            if (tentative_g < g_score[neighbor_idx]) {
                came_from[neighbor_idx] = current_idx;
                g_score[neighbor_idx] = tentative_g;
                double h = heuristic(neighbor, goal);
                f_score[neighbor_idx] = tentative_g + heuristic_weight_ * h;
                open_set.push({f_score[neighbor_idx], tentative_g, neighbor_idx});
            }
        }
    }
    
    search_stats_["expanded_nodes"] = expanded;
    search_stats_["iterations"] = iterations;
    search_stats_["success"] = 0;
    
    return {};
}

std::vector<std::tuple<int, int, int>> JPS3D::getSuccessors(
    const std::tuple<int, int, int>& current,
    const std::tuple<int, int, int>& /* parent */,
    const std::tuple<int, int, int>& /* goal */)
{
    std::vector<std::tuple<int, int, int>> successors;
    successors.reserve(8);

    auto [cx, cy, cz] = current;
    int search_range_z = static_cast<int>(std::ceil(max_z_change_ / voxel_map_->getResolution()));

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;

            int nx = cx + dx;
            int ny_ = cy + dy;

            if (voxel_map_->isSurfaceValid(nx, ny_, cz)) {
                successors.emplace_back(nx, ny_, cz);
                continue;
            }

            for (int dz_abs = 1; dz_abs <= search_range_z; ++dz_abs) {
                double height_diff = dz_abs * voxel_map_->getResolution();
                if (height_diff > max_z_change_) break;

                for (int sign : {-1, 1}) {
                    int nz = cz + sign * dz_abs;
                    if (voxel_map_->isSurfaceValid(nx, ny_, nz)) {
                        successors.emplace_back(nx, ny_, nz);
                        goto next_neighbor;
                    }
                }
            }
            next_neighbor:;
        }
    }
    return successors;
}

double JPS3D::heuristic(
    const std::tuple<int, int, int>& node1,
    const std::tuple<int, int, int>& node2) const
{
    auto [x1, y1, z1] = node1;
    auto [x2, y2, z2] = node2;

    // 使用距离场作为启发式 - 已经是整数距离
    int dist1 = voxel_map_->getDistanceToObstacle(x1, y1, z1);
    int dist2 = voxel_map_->getDistanceToObstacle(x2, y2, z2);
    
    // 基础距离：切比雪夫距离（整数运算，无sqrt）
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    int dz = std::abs(z1 - z2);
    int chebyshev = std::max({dx, dy, dz});
    
    // 启发式 = 切比雪夫距离 + 距离场差异（作为平滑项）
    return static_cast<double>(chebyshev) + std::abs(dist1 - dist2) * 0.1;
}

double JPS3D::distance(
    const std::tuple<int, int, int>& node1,
    const std::tuple<int, int, int>& node2) const
{
    auto [x1, y1, z1] = node1;
    auto [x2, y2, z2] = node2;
    
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    int dz = std::abs(z1 - z2);
    
    // 切比雪夫距离：对角移动代价为1
    double dist = static_cast<double>(std::max({dx, dy, dz}));
    
    // Z轴惩罚（可选）
    if (dz > 0) {
        dist += dz * z_penalty_weight_ * 0.1;
    }
    
    return dist;
}

std::vector<std::tuple<int, int, int>> JPS3D::reconstructPath(
    const std::unordered_map<std::tuple<int, int, int>, std::tuple<int, int, int>,
        TupleHash>& came_from,
    const std::tuple<int, int, int>& current)
{
    std::vector<std::tuple<int, int, int>> path;
    std::tuple<int, int, int> node = current;
    path.push_back(node);
    
    auto it = came_from.find(node);
    while (it != came_from.end()) {
        node = it->second;
        path.push_back(node);
        it = came_from.find(node);
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace jps_planner_3d