#include "jps_planner_3d/jps_3d.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_set>

namespace jps_planner_3d
{

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
, max_iterations_(50000)  // 降低到50K以提高响应速度
, cancel_planning_(false)
{
    // 地面机器人模式：使用Surface A*策略，不需要预生成搜索方向
    // getSuccessors() 直接搜索XY平面的8邻域
}

JPS3D::~JPS3D()
{
}

void JPS3D::cancelPlanning()
{
    cancel_planning_ = true;
}

std::vector<std::tuple<double, double, double>> JPS3D::plan(
	const std::tuple<double, double, double>& start_pos,
	const std::tuple<double, double, double>& goal_pos,
	bool snap_to_surface)
{
    // 重置取消标志，开始新的规划
    cancel_planning_ = false;

    // 清空之前的统计信息
    search_stats_.clear();

    // 转换为网格索引
    auto start_idx = voxel_map_->worldToGrid(std::get<0>(start_pos),
                                              std::get<1>(start_pos),
                                              std::get<2>(start_pos));
    auto goal_idx = voxel_map_->worldToGrid(std::get<0>(goal_pos),
                                             std::get<1>(goal_pos),
                                             std::get<2>(goal_pos));

    // 检查索引有效性
	if (!voxel_map_->isValidIdx(std::get<0>(start_idx), std::get<1>(start_idx), std::get<2>(start_idx)) ||
		!voxel_map_->isValidIdx(std::get<0>(goal_idx), std::get<1>(goal_idx), std::get<2>(goal_idx))) {
		search_stats_["failure_reason"] = 1; // 1 = invalid index
		return {};
	}

    // 地面机器人：直接使用 Surface 机制
    // 起点处理：确保位于有效的Surface上
    // Surface = BFS可达的地面体素上方的空气层
	auto [sx, sy, sz] = start_idx;
	if (!voxel_map_->isSurfaceValid(sx, sy, sz)) {
		if (!snap_to_surface) {
			search_stats_["failure_reason"] = 2; // 2 = start not on surface
			return {};
		}

		// 减小搜索半径以提高响应速度 (从5.0改为2.0)
		int search_radius = static_cast<int>(2.0 / voxel_map_->getResolution());

		// 传递取消检查回调，确保搜索过程可以被中断
		auto nearest = voxel_map_->findNearestValidSurface(
			sx, sy, sz, search_radius,
			[this]() { return cancel_planning_.load(); });

		if (std::get<0>(nearest) >= 0) {
			start_idx = nearest;
		} else {
			// 检查是否因为取消而失败
			if (cancel_planning_) {
				search_stats_["cancelled"] = 1;
				return {};
			}
			// 如果找不到有效Surface，直接返回失败
			search_stats_["failure_reason"] = 2; // 2 = no valid surface near start
			return {};
		}
	}

    // 终点处理同理
	auto [gx, gy, gz] = goal_idx;
	if (!voxel_map_->isSurfaceValid(gx, gy, gz)) {
		if (!snap_to_surface) {
			search_stats_["failure_reason"] = 3; // 3 = goal not on surface
			return {};
		}

		// 减小搜索半径以提高响应速度 (从5.0改为2.0)
		int search_radius = static_cast<int>(2.0 / voxel_map_->getResolution());

		// 传递取消检查回调，确保搜索过程可以被中断
		auto nearest = voxel_map_->findNearestValidSurface(
			gx, gy, gz, search_radius,
			[this]() { return cancel_planning_.load(); });

		if (std::get<0>(nearest) >= 0) {
			goal_idx = nearest;
		} else {
			// 检查是否因为取消而失败
			if (cancel_planning_) {
				search_stats_["cancelled"] = 1;
				return {};
			}
			search_stats_["failure_reason"] = 3; // 3 = no valid surface near goal
			return {};
		}
	}

    // 执行JPS搜索
    auto path_indices = jpsSearch(start_idx, goal_idx);

    if (path_indices.empty()) {
        return {};
    }

    // 转换为世界坐标
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
    // 优先队列: (f_score, counter, node)
    using QueueElement = std::tuple<double, int, std::tuple<int, int, int>>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> open_set;
    
    int counter = 0;
    open_set.push(std::make_tuple(0.0, counter++, start));

    // 记录节点信息
    std::unordered_map<std::tuple<int, int, int>, std::tuple<int, int, int>, TupleHash> came_from;
    std::unordered_map<std::tuple<int, int, int>, double, TupleHash> g_score;
    std::unordered_map<std::tuple<int, int, int>, double, TupleHash> f_score;

    g_score[start] = 0.0;
    f_score[start] = heuristic(start, goal);

    // 已访问集合
    std::unordered_set<std::tuple<int, int, int>, TupleHash> closed_set;

    // 迭代计数
    int iterations = 0;

    // 清空探索记录并预留空间，减少内存重新分配
    explored_nodes_.clear();
    explored_nodes_.reserve(1000);  // 预留1000个节点的空间

    while (!open_set.empty() && iterations < max_iterations_ && !cancel_planning_) {
        iterations++;
        auto [current_f, _, current] = open_set.top();
        open_set.pop();

        if (closed_set.find(current) != closed_set.end()) {
            continue;
        }

        // 记录探索的节点
        explored_nodes_.push_back(current);

        // 到达目标
        if (current == goal) {
            // 记录搜索统计
            search_stats_["explored_nodes"] = static_cast<int>(explored_nodes_.size());
            search_stats_["closed_nodes"] = static_cast<int>(closed_set.size());
            search_stats_["iterations"] = iterations;
            search_stats_["open_set_size"] = static_cast<int>(open_set.size());
            if (iterations > 0) {
                search_stats_["efficiency"] = static_cast<int>(closed_set.size() * 100.0 / iterations);
            } else {
                search_stats_["efficiency"] = 0;
            }
            return reconstructPath(came_from, current);
        }

        closed_set.insert(current);

        // 获取父节点
        std::tuple<int, int, int> parent(-1, -1, -1);
        if (came_from.find(current) != came_from.end()) {
            parent = came_from[current];
        }

        // 获取跳点邻居
        auto successors = getSuccessors(current, parent, goal);

        for (const auto& neighbor : successors) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            // 计算到邻居的代价
            // Python逻辑: tentative_g = g_score[current] + self._distance(current, neighbor)
            // 注意：Python _distance 调用 _heuristic，所以包含Z惩罚
            double dist = distance(current, neighbor);

            // 增加基于距离场的贴墙惩罚
            // 使用 Two-Pass DP 计算的距离值，距离越近惩罚越大
            if (potential_weight_ > 0.0) {
                auto [nx, ny, nz] = neighbor;
                int obstacle_dist = voxel_map_->getDistanceToObstacle(nx, ny, nz);
                
                // 距离场惩罚：使用反比函数，距离越小惩罚越大
                // penalty = potential_weight * resolution / (distance + 1)
                // 当 distance = 0 时（障碍物内），penalty = potential_weight * resolution
                // 当 distance = 1 时，penalty = potential_weight * resolution / 2
                // 当 distance = 2 时，penalty = potential_weight * resolution / 3
                // 以此类推，距离越远惩罚越小
                // 距离上限由 distance_field_max_distance 参数控制
                int max_distance = voxel_map_->getDistanceFieldMaxDistance();
                if (obstacle_dist > 0 && obstacle_dist <= max_distance) {
                    double penalty = potential_weight_ * voxel_map_->getResolution() / 
                                     static_cast<double>(obstacle_dist + 1);
                    dist += penalty;
                }
            }

            double tentative_g = g_score[current] + dist;

            // 地面机器人：检查Z轴变化约束
            auto [cx, cy, cz] = current;
            auto [nx, ny, nz] = neighbor;
            double z_change = std::abs(nz - cz) * voxel_map_->getResolution();
            // 如果Z变化过大，跳过此邻居
            if (z_change > max_z_change_ * 2) {
                continue;
            }

            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                // 应用启发式权重
                double h = heuristic(neighbor, goal);
                double f = tentative_g + heuristic_weight_ * h;
                f_score[neighbor] = f;
                open_set.push(std::make_tuple(f, counter++, neighbor));
            }
        }
    }
    
    // 检查循环退出原因
    if (cancel_planning_) {
        // 规划被取消
        search_stats_["explored_nodes"] = static_cast<int>(explored_nodes_.size());
        search_stats_["closed_nodes"] = static_cast<int>(closed_set.size());
        search_stats_["iterations"] = iterations;
        search_stats_["open_set_size"] = static_cast<int>(open_set.size());
        search_stats_["success"] = 0;
        search_stats_["cancelled"] = 1;
        return {};
    }

    // 如果一直找不到路径，可能是因为起点/终点虽然经过了吸附，但所在的连通域不连通
    // 或者中间存在无法逾越的障碍（如楼梯太高）
    if (explored_nodes_.size() > 100 && closed_set.size() > 10) {
        // 可以在这里添加一些调试日志，查看是否因为max_z_change限制太死导致无法扩展
        // 但在纯算法库中不便添加ROS日志，这通常由调用者处理统计信息
    }

    // 超过最大迭代次数或未找到路径
    search_stats_["explored_nodes"] = static_cast<int>(explored_nodes_.size());
    search_stats_["closed_nodes"] = static_cast<int>(closed_set.size());
    search_stats_["iterations"] = iterations;
    search_stats_["open_set_size"] = static_cast<int>(open_set.size());
    search_stats_["success"] = 0;
    search_stats_["cancelled"] = 0;

    return {};
}

std::vector<std::tuple<int, int, int>> JPS3D::getSuccessors(
    const std::tuple<int, int, int>& current,
    const std::tuple<int, int, int>& /* parent */,
    const std::tuple<int, int, int>& /* goal */)
{
    std::vector<std::tuple<int, int, int>> successors;
    successors.reserve(8);  // 预留8个邻居的空间

    // 地面机器人使用Surface A*策略（符合2D+图层逻辑）
    auto [cx, cy, cz] = current;
    int search_range_z = static_cast<int>(std::ceil(max_z_change_ / voxel_map_->getResolution()));

    // 搜索8邻域（XY平面）
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;

            // 检查取消标志（每处理一个XY邻居检查一次）
            if (cancel_planning_.load()) {
                return successors;  // 立即返回已找到的邻居
            }

            int nx = cx + dx;
            int ny = cy + dy;

            // 优化的Z方向搜索策略：
            // 1. 先检查当前Z层 (dz=0)
            // 2. 然后螺旋向外搜索 (dz=±1, ±2, ...)
            // 3. 找到第一个有效surface就使用（优先高度差小的）

            // 先检查同一高度
            if (voxel_map_->isSurfaceValid(nx, ny, cz)) {
                successors.emplace_back(nx, ny, cz);
                continue;  // 找到最优（高度差为0），直接处理下一个XY邻居
            }

            // 螺旋向上下搜索
            bool found_surface = false;
            for (int dz_abs = 1; dz_abs <= search_range_z; ++dz_abs) {
                double height_diff = dz_abs * voxel_map_->getResolution();
                if (height_diff > max_z_change_) {
                    break;  // 超出高度限制，停止搜索
                }

                // 先检查下方，再检查上方（优先下降）
                for (int sign : {-1, 1}) {
                    int nz = cz + sign * dz_abs;
                    if (voxel_map_->isSurfaceValid(nx, ny, nz)) {
                        successors.emplace_back(nx, ny, nz);
                        found_surface = true;
                        break;  // 找到有效surface，停止Z搜索
                    }
                }
                if (found_surface) break;
            }
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

    // 基础欧几里得距离（网格坐标）
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dz = z1 - z2;
    double h_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 地面机器人：对Z轴变化添加额外惩罚
    double z_change = std::abs(dz) * voxel_map_->getResolution();
    // 对超过阈值的Z变化施加惩罚
    if (z_change > max_z_change_) {
        double z_penalty = (z_change - max_z_change_) * z_penalty_weight_;
        h_dist += z_penalty;
    } else {
        // 即使在阈值内，也轻微惩罚Z变化
        h_dist += std::abs(dz) * 0.5;
    }

    return h_dist;
}

double JPS3D::distance(
    const std::tuple<int, int, int>& node1,
    const std::tuple<int, int, int>& node2) const
{
    // Python版本中，_distance 调用 _heuristic
    // 这样 G score 就包含了几何距离（网格单位）和 Z轴惩罚（米单位）
    // 虽然单位混合，但为了保持与Python版本行为一致，必须这样做
    return heuristic(node1, node2);
}

std::vector<std::tuple<int, int, int>> JPS3D::reconstructPath(
    const std::unordered_map<std::tuple<int, int, int>, std::tuple<int, int, int>, TupleHash>& came_from,
    const std::tuple<int, int, int>& current)
{
    std::vector<std::tuple<int, int, int>> path;
    std::tuple<int, int, int> node = current;
    path.push_back(node);

    while (came_from.find(node) != came_from.end()) {
        node = came_from.at(node);
        path.push_back(node);
    }

    std::reverse(path.begin(), path.end());
    return path;
}


}  // namespace jps_planner_3d