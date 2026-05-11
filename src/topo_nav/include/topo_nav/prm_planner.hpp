#pragma once

#include "topo_nav/hierarchical_planner.hpp"   // HierarchicalPath

#include <memory>
#include <tuple>
#include <vector>

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

/// PRM (Probabilistic Roadmap) baseline planner on the surface graph.
class PRMPlanner
{
public:
  PRMPlanner(std::shared_ptr<jps_planner_3d::VoxelMap> vmap,
             double w_up, double w_down, double w_obs);

  /// Build the PRM roadmap by sampling surface nodes.
  /// @param num_samples   Number of roadmap nodes to sample
  /// @param k_neighbors   Max neighbors per node
  /// @param max_edge_len  Max edge length in meters (0 = no limit)
  void buildRoadmap(int num_samples, int k_neighbors, double max_edge_len);

  /// Plan using the PRM roadmap (Dijkstra + local A* for first/last mile).
  HierarchicalPath plan(
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    float robot_diameter_m = 0.4f,
    bool snap_to_surface = true);

  double getBuildTimeMs() const { return build_time_ms_; }

private:
  struct RoadmapEdge { int to; double cost; };

  std::shared_ptr<jps_planner_3d::VoxelMap> vmap_;
  double w_up_, w_down_, w_obs_;

  std::vector<int> node_surf_;                         // roadmap node → surface index
  std::vector<std::vector<RoadmapEdge>> adj_;          // adjacency list
  int num_nodes_ = 0;
  double build_time_ms_ = 0.0;

  // Dirty-array A* state for local planning
  mutable std::vector<double>   astar_g_;
  mutable std::vector<int>      astar_came_from_;
  mutable std::vector<uint32_t> astar_g_gen_;
  mutable std::vector<uint32_t> astar_cl_gen_;
  mutable uint32_t              astar_gen_ = 0;

  int snapToSurface(const std::tuple<double, double, double> & world, bool snap) const;
  double edgeCost(int s1, int s2) const;
  std::vector<int> localAstar(int start_surf, int goal_surf, int* out_expanded = nullptr) const;
};

}  // namespace topo_nav
