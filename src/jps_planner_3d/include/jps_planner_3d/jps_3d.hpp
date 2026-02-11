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
    JPS3D(
        std::shared_ptr<VoxelMap> voxel_map,
        double heuristic_weight = 1.0,
        double max_z_change = 0.5,
        double z_penalty_weight = 3.0,
        double potential_weight = 100.0);

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
    double max_z_change_;
    double z_penalty_weight_;
    int max_iterations_;
    int expanded_nodes_ = 0;
    int iterations_ = 0;
};

}  // namespace jps_planner_3d

#endif  // JPS_PLANNER_3D__JPS_3D_HPP_
