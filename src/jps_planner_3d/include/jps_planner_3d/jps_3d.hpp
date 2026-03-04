#ifndef JPS_PLANNER_3D__JPS_3D_HPP_
#define JPS_PLANNER_3D__JPS_3D_HPP_

#include "jps_planner_3d/voxel_map.hpp"
#include <vector>
#include <tuple>
#include <memory>

namespace jps_planner_3d
{

class JPS3D
{
public:
    // 论文参数: ε=heuristic_weight, ω_↑=w_up, ω_↓=w_down, ω_obs=w_obs
    JPS3D(
        std::shared_ptr<VoxelMap> voxel_map,
        double heuristic_weight = 1.0,
        double w_up = 2.0,
        double w_down = 1.0,
        double w_obs = 0.5);

    ~JPS3D();

    std::vector<std::tuple<double, double, double>> plan(
        const std::tuple<double, double, double>& start_pos,
        const std::tuple<double, double, double>& goal_pos,
        bool snap_to_surface = true);

    int getExpandedNodes() const { return expanded_nodes_; }
    int getIterations() const { return iterations_; }

private:
    std::vector<int> searchOnGraph(int start_idx, int goal_idx);
    double heuristic(int node1_idx, int node2_idx) const;
    double distance(int node1_idx, int node2_idx) const;

    std::shared_ptr<VoxelMap> voxel_map_;
    double heuristic_weight_;
    double w_up_;    // 上坡惩罚权重 ω_↑ (论文: 2.0)
    double w_down_;  // 下坡惩罚权重 ω_↓ (论文: 1.0)
    double w_obs_;   // 障碍物间隙权重 ω_obs (论文: 0.5)
    int expanded_nodes_ = 0;
    int iterations_ = 0;
};

}  // namespace jps_planner_3d

#endif  // JPS_PLANNER_3D__JPS_3D_HPP_
