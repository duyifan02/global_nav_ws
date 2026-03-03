#include "jps_planner_3d/jps_3d.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace jps_planner_3d
{

JPS3D::JPS3D(
    std::shared_ptr<VoxelMap> voxel_map,
    double heuristic_weight,
    double w_up,
    double w_down,
    double w_obs)
: voxel_map_(voxel_map)
, heuristic_weight_(heuristic_weight)
, w_up_(w_up)
, w_down_(w_down)
, w_obs_(w_obs)
, max_iterations_(50000)
{
}

JPS3D::~JPS3D()
{
}

std::vector<std::tuple<double, double, double>> JPS3D::plan(
    const std::tuple<double, double, double>& start_pos,
    const std::tuple<double, double, double>& goal_pos,
    bool snap_to_surface)
{
    auto start_coord = voxel_map_->worldToGrid(std::get<0>(start_pos),
                                                std::get<1>(start_pos),
                                                std::get<2>(start_pos));
    auto goal_coord = voxel_map_->worldToGrid(std::get<0>(goal_pos),
                                               std::get<1>(goal_pos),
                                               std::get<2>(goal_pos));

    auto [sx, sy, sz] = start_coord;
    auto [gx, gy, gz] = goal_coord;

    if (!voxel_map_->isValidIdx(sx, sy, sz) || !voxel_map_->isValidIdx(gx, gy, gz)) {
        return {};
    }

    int start_idx = voxel_map_->coordToSurfaceIdx(sx, sy, sz);
    if (start_idx < 0) {
        if (!snap_to_surface) {
            return {};
        }
        int search_radius = static_cast<int>(5.0 / voxel_map_->getResolution());
        auto nearest = voxel_map_->findNearestValidSurface(sx, sy, sz, search_radius);
        if (std::get<0>(nearest) < 0) {
            return {};
        }
        auto [nx, ny, nz] = nearest;
        start_idx = voxel_map_->coordToSurfaceIdx(nx, ny, nz);
    }

    int goal_idx = voxel_map_->coordToSurfaceIdx(gx, gy, gz);
    if (goal_idx < 0) {
        if (!snap_to_surface) {
            return {};
        }
        int search_radius = static_cast<int>(5.0 / voxel_map_->getResolution());
        auto nearest = voxel_map_->findNearestValidSurface(gx, gy, gz, search_radius);
        if (std::get<0>(nearest) < 0) {
            return {};
        }
        auto [nx, ny, nz] = nearest;
        goal_idx = voxel_map_->coordToSurfaceIdx(nx, ny, nz);
    }

    auto path_indices = searchOnGraph(start_idx, goal_idx);

    if (path_indices.empty()) {
        return {};
    }

    std::vector<std::tuple<double, double, double>> path;
    const auto& graph = voxel_map_->getSurfaceGraph();
    for (int idx : path_indices) {
        const auto& node = graph[idx];
        auto world_pos = voxel_map_->gridToWorld(node.x, node.y, node.z);
        path.push_back(world_pos);
    }

    return path;
}

std::vector<int> JPS3D::searchOnGraph(int start_idx, int goal_idx)
{
    const auto& graph = voxel_map_->getSurfaceGraph();
    const auto& all_neighbors = voxel_map_->getAllNeighbors();
    const int n = static_cast<int>(graph.size());
    
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(n, INF);
    std::vector<double> f_score(n, INF);
    std::vector<int> came_from(n, -1);
    std::vector<char> closed(n, 0);
    
    using QueueElement = std::tuple<double, double, int>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> open_set;
    
    g_score[start_idx] = 0.0;
    f_score[start_idx] = heuristic(start_idx, goal_idx);
    open_set.emplace(f_score[start_idx], 0.0, start_idx);
    
    iterations_ = 0;
    expanded_nodes_ = 0;
    
    while (!open_set.empty() && iterations_ < max_iterations_) {
        iterations_++;
        auto [current_f, current_g, current_idx] = open_set.top();
        open_set.pop();
        
        if (closed[current_idx]) continue;
        
        if (current_idx == goal_idx) {
            std::vector<int> path;
            int node = goal_idx;
            while (node >= 0) {
                path.push_back(node);
                if (node == start_idx) break;
                node = came_from[node];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        closed[current_idx] = 1;
        expanded_nodes_++;
        
        const auto& node = graph[current_idx];
        for (int i = 0; i < node.neighbor_count; ++i) {
            int neighbor_idx = all_neighbors[node.neighbor_start + i];
            
            if (closed[neighbor_idx]) continue;
            
            double dist = distance(current_idx, neighbor_idx);
            double tentative_g = g_score[current_idx] + dist;
            
            if (tentative_g < g_score[neighbor_idx]) {
                came_from[neighbor_idx] = current_idx;
                g_score[neighbor_idx] = tentative_g;
                double h = heuristic(neighbor_idx, goal_idx);
                f_score[neighbor_idx] = tentative_g + heuristic_weight_ * h;
                open_set.emplace(f_score[neighbor_idx], tentative_g, neighbor_idx);
            }
        }
    }
    
    return {};
}

double JPS3D::heuristic(int node1_idx, int node2_idx) const
{
    const auto& graph = voxel_map_->getSurfaceGraph();
    const auto& n1 = graph[node1_idx];
    const auto& n2 = graph[node2_idx];

    int dx = std::abs(n1.x - n2.x);
    int dy = std::abs(n1.y - n2.y);
    int dz = std::abs(n1.z - n2.z);

    double r = voxel_map_->getResolution();

    // 论文公式(11): h(s) = r*‖s-s_goal‖₂ + r*|Δz_g|*ω_↓
    // 可容许性: Euclidean下界路径长度，ω_↓ ≤ ω_↑ 保证z代价不高估
    double eucl = r * std::sqrt(static_cast<double>(dx*dx + dy*dy + dz*dz));
    double z_cost = dz * r * w_down_;

    return eucl + z_cost;
}

double JPS3D::distance(int node1_idx, int node2_idx) const
{
    const auto& graph = voxel_map_->getSurfaceGraph();
    const auto& n1 = graph[node1_idx];
    const auto& n2 = graph[node2_idx];

    int dx = std::abs(n1.x - n2.x);
    int dy = std::abs(n1.y - n2.y);
    int dz_signed = n2.z - n1.z;  // 有符号：区分上坡/下坡
    int dz = std::abs(dz_signed);

    double r = voxel_map_->getResolution();

    // 论文公式(10): c(s,s') = r*‖s'-s‖₂ + |Δz|*r*ω_z + ω_obs*r/(D(s')+1)
    // ω_z ∈ {ω_↑, ω_↓, 0}，上坡用ω_↑，下坡用ω_↓，水平用0
    double eucl = r * std::sqrt(static_cast<double>(dx*dx + dy*dy + dz*dz));
    double wz = (dz_signed > 0) ? w_up_ : ((dz_signed < 0) ? w_down_ : 0.0);
    double z_cost = dz * r * wz;
    // 贴墙惩罚: D(s')=0(紧贴障碍)时最大，D越大惩罚越小，鼓励走空旷区域
    double clearance_cost = w_obs_ * r / (n2.distance + 1);

    return eucl + z_cost + clearance_cost;
}

} // namespace jps_planner_3d
