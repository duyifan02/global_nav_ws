#pragma once

#include "topo_nav/room_segmenter.hpp"
#include "topo_nav/floor_detector.hpp"

#include <memory>
#include <tuple>
#include <vector>
#include <string>

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

struct HierarchicalPath
{
  bool success = false;
  std::string message;

  /// Complete path as world-coordinate waypoints (voxel-level resolution).
  std::vector<std::tuple<double, double, double>> waypoints;

  std::vector<int> room_sequence;   ///< Ordered room ids traversed
  std::vector<int> floor_sequence;  ///< Ordered floor ids traversed

  double floor_planning_ms  = 0.0;
  double room_planning_ms   = 0.0;
  double voxel_planning_ms  = 0.0;
  double total_planning_ms  = 0.0;

  double path_length_m      = 0.0;
  int voxel_nodes_expanded  = 0;
};

/// Three-level hierarchical A* planner.
///
/// Level 0 = voxel (surface graph)
/// Level 1 = room (RoomGraph)
/// Level 2 = floor (FloorGraph)
class HierarchicalPlanner
{
public:
  HierarchicalPlanner(
    std::shared_ptr<jps_planner_3d::VoxelMap> vmap,
    double w_up   = 2.0,
    double w_down = 1.0,
    double w_obs  = 0.5);

  /// Load topological map.  Must be called before plan().
  void setTopoMap(const RoomGraph & rg, const FloorGraph & fg);

  bool hasTopoMap() const { return has_topo_map_; }

  /// Plan from start to goal.
  ///
  /// @param start_world      World coordinate of start
  /// @param goal_world       World coordinate of goal
  /// @param robot_diameter_m Passage-width filter (metres)
  /// @param snap_to_surface  Snap start/goal to nearest surface voxel
  HierarchicalPath plan(
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    float  robot_diameter_m = 0.4f,
    bool   snap_to_surface  = true);

private:
  // ── Sub-planners ──────────────────────────────────────────────────────────

  /// Step 1: A* on FloorGraph.  Returns ordered floor ids, empty on failure.
  std::vector<int> floorAstar(int start_floor, int goal_floor) const;

  /// Step 2: A* on RoomGraph restricted to relevant floors + staircase rooms.
  /// Returns ordered room ids, empty on failure.
  std::vector<int> roomAstar(
    int start_room,
    int goal_room,
    const std::vector<int> & allowed_floors,
    float robot_diameter_m) const;

  /// Step 3: Voxel-level A* on surface graph.
  /// If allowed_room_id >= 0, only nodes in that room are expanded
  /// (start_surf and goal_surf are always accessible).
  /// Returns ordered surface-graph indices, empty on failure.
  std::vector<int> voxelAstar(
    int start_surf,
    int goal_surf,
    int allowed_room_id = -1) const;

  /// Snap world coordinate to surface graph index.  Returns -1 on failure.
  int snapToSurface(
    const std::tuple<double, double, double> & world,
    bool snap) const;

  double computePathLength(const std::vector<int> & surf_indices) const;

  // ── State ─────────────────────────────────────────────────────────────────
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap_;
  double w_up_, w_down_, w_obs_;

  RoomGraph  rg_;
  FloorGraph fg_;
  bool has_topo_map_ = false;
};

}  // namespace topo_nav
