#pragma once

#include "topo_nav/hierarchical_planner.hpp"   // HierarchicalPath

#include <memory>
#include <tuple>
#include <vector>

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

/// ALT (A*, Landmarks, Triangle inequality) baseline planner on the surface graph.
///
/// Precomputes single-source distances from L landmarks, then uses
/// h(v) = max_l |d(l, goal) - d(l, v)| as an admissible, tighter-than-Euclidean
/// heuristic for A*.
class ALTPlanner
{
public:
  ALTPlanner(std::shared_ptr<jps_planner_3d::VoxelMap> vmap,
             double w_up, double w_down, double w_obs);

  /// Precompute landmark distances (L full Dijkstras on the surface graph).
  void precompute(int num_landmarks = 16);

  /// Plan using A* with landmark heuristic.
  HierarchicalPath plan(
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    float robot_diameter_m = 0.4f,
    bool snap_to_surface = true);

  double getPrecomputeTimeMs() const { return precompute_time_ms_; }

private:
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap_;
  double w_up_, w_down_, w_obs_;

  int num_landmarks_ = 0;
  int num_surface_ = 0;
  std::vector<int> landmark_surf_;        // landmark surface indices
  std::vector<double> landmark_dist_;     // [l * num_surface_ + v] = dist from landmark l to v
  double precompute_time_ms_ = 0.0;

  int snapToSurface(const std::tuple<double, double, double> & world, bool snap) const;
  double edgeCost(int s1, int s2) const;
  double heuristic(int v, int goal) const;
  double computePathLength(const std::vector<int> & path) const;
};

}  // namespace topo_nav
