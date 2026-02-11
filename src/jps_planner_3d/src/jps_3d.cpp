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
    double max_z_change,
    double z_penalty_weight,
    double /* potential_weight */)
: voxel_map_(voxel_map)
, heuristic_weight_(heuristic_weight)
, max_z_change_(max_z_change)
, z_penalty_weight_(z_penalty_weight)
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
    int chebyshev = std::max({dx, dy, dz});
    
    return static_cast<double>(chebyshev) + std::abs(n1.distance - n2.distance) * 2.0;
}

double JPS3D::distance(int node1_idx, int node2_idx) const
{
    const auto& graph = voxel_map_->getSurfaceGraph();
    const auto& n1 = graph[node1_idx];
    const auto& n2 = graph[node2_idx];
    
    int dx = std::abs(n1.x - n2.x);
    int dy = std::abs(n1.y - n2.y);
    int dz = std::abs(n1.z - n2.z);
    
    double dist = static_cast<double>(std::max({dx, dy, dz}));
    
    if (dz > 0) {
        dist += dz * z_penalty_weight_ * 0.1;
    }
    
    return dist;
}

} // namespace jps_planner_3d
