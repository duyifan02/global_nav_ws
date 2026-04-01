#pragma once

#include "topo_nav/room_segmenter.hpp"
#include "topo_nav/floor_detector.hpp"

#include <cstdint>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
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
  /// start_world / goal_world are used for admissible init-cost and heuristic.
  std::vector<int> roomAstar(
    int start_room,
    int goal_room,
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    const std::vector<int> & allowed_floors,
    float robot_diameter_m) const;

  /// Step 3: Voxel-level A* on surface graph.
  /// Room filter (mutually exclusive, corridor takes priority):
  ///   corridor != nullptr  → only voxels whose room_label ∈ *corridor
  ///   allowed_room_id >= 0 → only voxels in that single room
  ///   both null / -1       → unrestricted search
  /// start_surf and goal_surf are always accessible regardless of filter.
  std::vector<int> voxelAstar(
    int start_surf,
    int goal_surf,
    int allowed_room_id = -1,
    int* out_expanded   = nullptr,
    const std::unordered_set<int>* corridor = nullptr) const;

  /// Exhaustive single-source Dijkstra restricted to one room.
  /// Populates out_dist[v] = shortest cost from source to v,
  /// out_parent[v] = predecessor on that path, for all reachable v in room.
  void roomDijkstra(
    int source_surf,
    int room_id,
    std::unordered_map<int, double> & out_dist,
    std::unordered_map<int, int>    & out_parent) const;

  /// Snap world coordinate to surface graph index.  Returns -1 on failure.
  int snapToSurface(
    const std::tuple<double, double, double> & world,
    bool snap) const;

  double computePathLength(const std::vector<int> & surf_indices) const;

  // ── HPA* precomputation ───────────────────────────────────────────────────

  /// Precompute all pairwise intra-room transfers.  Called by setTopoMap().
  void precomputeTransfers();

  /// Flat transfer table indexed by [e_in * transfer_E_ + e_out].
  ///
  /// transfer_E_ = total number of room edges (rg_.edges.size()).
  /// transfer_cost_[e_in * E + e_out]  = actual voxel path cost (INF = no path).
  /// transfer_path_[e_in * E + e_out]  = surface-graph indices (empty = no path).
  ///
  /// Space: E² × (8 B cost + sizeof(vector) path) ≈ 40 KB for E=50.
  int transfer_E_ = 0;
  std::vector<double>           transfer_cost_;   ///< [e_in * E + e_out], INF = no path
  std::vector<std::vector<int>> transfer_path_;   ///< [e_in * E + e_out], empty = no path

  /// Flat minimum exit cost table indexed by [room_id * transfer_E_ + edge_out].
  /// Stores the minimum actual voxel traversal cost within room r to reach
  /// edge_out, minimised over all possible entry edges into room r.
  /// Used by roomAstar() for tighter (precomputed) edge-cost estimates.
  ///
  /// Space: R × E × 8 B ≈ 7 KB for R=18, E=50.
  std::vector<double> min_exit_cost_flat_;  ///< [room * E + edge_out], INF = no path

  // ── Scheme 2: Transition distance fields (built by roomDijkstra) ──────────
  //
  // For each transition surface voxel t, stores the complete single-source
  // shortest-path tree from t to all reachable voxels in t's room.
  //
  // Keyed by transition surf index (unique per room-side of an edge).
  // Enables O(1) cost lookup and O(path) reconstruction at query time.
  //
  // Memory: per-transition ≈ room_voxels × 12 B; typical total ≈ 2–4 MB.
  struct TransitionField {
    std::unordered_map<int, double> dist;    ///< surf_idx → cost from transition
    std::unordered_map<int, int>    parent;  ///< surf_idx → predecessor
  };
  std::unordered_map<int, TransitionField> trans_field_;  ///< key = transition surf_idx

  // ── Dirty-array A* state (space-for-time: eliminates per-call O(n) init) ──
  //
  // Instead of zeroing 3 × n-element arrays before every voxelAstar() call
  // (650 KB × 108 calls = 70 MB of memset during precomputeTransfers()),
  // we maintain persistent arrays and a monotone generation counter.
  //
  // A cell i is "initialised in the current call" iff
  //   astar_g_gen_[i]  == astar_gen_   (for g and came_from)
  //   astar_cl_gen_[i] == astar_gen_   (for the closed set)
  //
  // Bumping the counter (O(1)) "resets" all cells without touching them.
  // The arrays are allocated once in setTopoMap() and reused forever.
  //
  // Extra space: 2 × n × 4 B = 400 KB for n = 50 000.
  mutable std::vector<double>   astar_g_;           ///< g-cost (valid only if gen matches)
  mutable std::vector<int>      astar_came_from_;   ///< predecessor (valid if gen matches)
  mutable std::vector<uint32_t> astar_g_gen_;       ///< generation when g/came_from written
  mutable std::vector<uint32_t> astar_cl_gen_;      ///< generation when node closed
  mutable uint32_t              astar_gen_ = 0;     ///< current generation counter

  // ── State ─────────────────────────────────────────────────────────────────
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap_;
  double w_up_, w_down_, w_obs_;

  RoomGraph  rg_;
  FloorGraph fg_;
  bool has_topo_map_ = false;
};

}  // namespace topo_nav
