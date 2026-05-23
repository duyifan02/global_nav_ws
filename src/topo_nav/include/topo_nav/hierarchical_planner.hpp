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

/// Per-query diagnostic data for PH-ALT-TG analysis.
struct QueryDiagnostics
{
  // ── Room-level routing ──
  int    start_room        = -1;
  int    goal_room         = -1;
  bool   same_room         = false;
  int    room_hops         = 0;     ///< Number of room transitions in route

  // ── TG A* search stats ──
  int    tg_expanded       = 0;     ///< Nodes popped from TG A* open list
  double tg_astar_ms       = 0.0;   ///< TG A* wall-clock time
  double h_start           = 0.0;   ///< ALT heuristic at start's first portal
  double tg_cost           = 0.0;   ///< Optimal TG cost (g[VGOAL])

  // ── Stitch / reconstruction ──
  int    stitch_segments   = 0;     ///< Number of stitched segments
  int    stitch_cached     = 0;     ///< Segments served from cache
  int    stitch_fallback   = 0;     ///< Segments requiring online A*
  double stitch_ms         = 0.0;   ///< Stitch wall-clock time
  int    fallback_nodes    = 0;     ///< Total voxel expansions in fallback A*
  std::string fallback_reason;      ///< Reason for first fallback (if any)

  // ── Cache diagnostics ──
  bool   cache_hit_all     = true;  ///< All segments served from precomputed data

  // ── Connectivity ──
  int    tg_nodes          = 0;     ///< |V_T| at query time
  int    start_portals     = 0;     ///< Portals in start room
  int    goal_portals      = 0;     ///< Portals in goal room

  // ── P0.3: Fallback timing ──
  double fallback_astar_ms = 0.0;   ///< Time spent in fallback voxelAstar segments
};

/// Scene-level precompute statistics (populated during BuildTopoMap).
struct PrecomputeStats
{
  int    num_rooms             = 0;
  int    num_edges             = 0;
  int    num_portals           = 0;    ///< Total portal voxels across all edges
  double portal_k_effective    = 0.0;  ///< Mean portals per side per edge
  int    tg_nodes              = 0;
  int    tg_edges              = 0;
  int    tg_landmarks          = 0;
  int    tg_connected_components = 0;
  int    max_room_voxels       = 0;
  int    max_room_portals      = 0;
  double max_room_fraction     = 0.0;  ///< max_room_voxels / |V_surface|
  int    skipped_rooms         = 0;    ///< Rooms skipped by budget cap
  double skipped_voxel_fraction = 0.0; ///< Fraction of voxels in skipped rooms
  int    transfer_pairs_total  = 0;    ///< Expected transfer pairs
  int    transfer_pairs_cached = 0;    ///< Actually cached pairs
  double transfer_coverage     = 0.0;  ///< cached / total
};

struct HierarchicalPath
{
  bool success = false;
  std::string message;

  /// Complete path as world-coordinate waypoints (voxel-level resolution).
  std::vector<std::tuple<double, double, double>> waypoints;

  std::vector<int> room_sequence;   ///< Ordered room ids traversed
  std::vector<int> floor_sequence;  ///< Ordered floor ids traversed

  double snap_ms            = 0.0;
  double floor_planning_ms  = 0.0;
  double room_planning_ms   = 0.0;
  double voxel_planning_ms  = 0.0;
  double total_planning_ms  = 0.0;

  double path_length_m      = 0.0;
  int voxel_nodes_expanded  = 0;

  // ── P0.4: Failure reason classification ──
  std::string failure_reason = "ok";   ///< ok, snap_failed, tg_astar_failed, stitch_failed, etc.

  // ── P0.2: Path validity ──
  bool path_valid               = false;
  std::string invalid_reason;
  int  invalid_edge_count       = 0;
  double max_step_distance_voxels = 0.0;

  QueryDiagnostics diag;  ///< Detailed per-query diagnostics
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
  /// @param skip_precompute  If true, skip precomputeTransfers() (for no_stitch ablation)
  void setTopoMap(const RoomGraph & rg, const FloorGraph & fg,
                  bool skip_precompute = false);

  bool hasTopoMap() const { return has_topo_map_; }

  /// Enable/disable stitchPath at query time (for no_stitch ablation).
  void setUseStitch(bool v) { use_stitch_ = v; }

  /// Enable PH-ALT-TG mode: use transition graph A* instead of roomAstar.
  void setUsePHALT(bool v) { use_ph_alt_ = v; }

  /// Precompute ALT landmarks on G_T via farthest-point Dijkstra.
  /// Must be called after setTopoMap() (which builds the transition graph).
  void precomputeTGLandmarks(int num_landmarks);

  /// Get scene-level precompute statistics (valid after setTopoMap).
  const PrecomputeStats & getPrecomputeStats() const { return precompute_stats_; }

  /// Plan from start to goal (requires setTopoMap).
  HierarchicalPath plan(
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    float  robot_diameter_m = 0.4f,
    bool   snap_to_surface  = true);

  /// Flat surface A* (no hierarchy).  Only requires a loaded VoxelMap.
  HierarchicalPath planFlat(
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    float  robot_diameter_m = 0.4f,
    bool   snap_to_surface  = true);

private:
  // ── Sub-planners ──────────────────────────────────────────────────────────

  /// Step 1: A* on FloorGraph.  Returns ordered floor ids, empty on failure.
  std::vector<int> floorAstar(int start_floor, int goal_floor) const;

  /// Result of room-level routing.
  struct RoomRoute {
    std::vector<int> rooms;          ///< Ordered room ids (size = N+1, including start/goal)
    std::vector<int> edges;          ///< Ordered edge ids between consecutive rooms (size = N)
    /// Multi-portal: specific portal surf_idx chosen by TG A* for each crossing.
    /// exit_portals[i]  = surf_idx in rooms[i]   (exit side of edges[i])
    /// entry_portals[i] = surf_idx in rooms[i+1] (entry side of edges[i])
    /// Empty when route comes from roomAstar (falls back to primary portal).
    std::vector<int> exit_portals;
    std::vector<int> entry_portals;
  };

  /// Step 2: A* on RoomGraph restricted to relevant floors + staircase rooms.
  /// Returns ordered (rooms, edges); rooms.empty() on failure.
  ///
  /// start_surf / goal_surf are used to query precomputed trans_field_ for
  /// EXACT geodesic init/admission costs (when available).  This makes the
  /// chosen room sequence HPA*-optimal w.r.t. the precomputed transitions
  /// rather than relying on a loose Euclidean lower bound.
  RoomRoute roomAstar(
    int start_room,
    int goal_room,
    int start_surf,
    int goal_surf,
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    const std::vector<int> & allowed_floors,
    float robot_diameter_m) const;

  /// Step 3 (new): assemble surface path from precomputed segments.
  ///
  /// For each room segment, prefers O(1) trans_field_ parent-chain walk.
  /// Falls back to single-room voxelAstar for any room whose trans_field_ was
  /// skipped during precompute (> kMaxEdgesForDijkstra edges).
  ///
  /// @param out_expanded  Accumulates voxel expansions from fallback segments.
  ///                      0 on a pure-lookup (all cached) path.
  /// @returns empty only if a per-segment voxelAstar fails (unreachable).
  std::vector<int> stitchPath(
    int start_surf,
    int goal_surf,
    const RoomRoute & route,
    int * out_expanded = nullptr,
    const std::unordered_set<int>* fallback_corridor = nullptr,
    QueryDiagnostics * diag = nullptr) const;

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

  /// Single-source Dijkstra for intra-room precomputation.
  ///
  /// Runs an UNRESTRICTED Dijkstra (no room filter) from source_surf,
  /// but early-terminates once all room-interior voxels are covered.
  /// This allows the wave to bridge through narrow necks of adjacent rooms
  /// where a transition voxel has zero same-room neighbours.
  ///
  /// Results are extracted ONLY for voxels in room_id (+ the source).
  /// Paths stored in the parent chain may transiently cross room boundaries;
  /// that is geometrically correct and typically adds < 3 voxels of detour.
  ///
  /// @param room_voxel_count  Number of voxels with room_labels==room_id.
  ///                          Used for early termination heuristic.
  void roomDijkstra(
    int source_surf,
    int room_id,
    int room_voxel_count,
    std::unordered_map<int, double> & out_dist,
    std::unordered_map<int, int>    & out_parent) const;

  /// Snap world coordinate to surface graph index.  Returns -1 on failure.
  int snapToSurface(
    const std::tuple<double, double, double> & world,
    bool snap) const;

  double computePathLength(const std::vector<int> & surf_indices) const;

  /// Validate stitched path: 26-neighbor connectivity, surface membership.
  /// Populates result.path_valid, result.invalid_reason, etc.
  void validatePath(const std::vector<int> & surf_path,
                    int start_surf, int goal_surf,
                    HierarchicalPath & result) const;

  // ── HPA* precomputation ───────────────────────────────────────────────────

  /// Precompute all pairwise intra-room transfers.  Called by setTopoMap().
  void precomputeTransfers();

  /// Flat transfer cost table indexed by [e_in * transfer_E_ + e_out].
  ///
  /// transfer_E_ = total number of room edges (rg_.edges.size()).
  /// transfer_cost_[e_in * E + e_out] = actual voxel path cost (INF = no path).
  /// Path reconstruction uses trans_field_ parent-chain walks instead of
  /// materializing all pair paths (root-Dijkstra policy, saves O(k²|R|) memory).
  ///
  /// Space: E² × 8 B ≈ 20 KB for E=50.
  int transfer_E_ = 0;
  std::vector<double> transfer_cost_;   ///< [e_in * E + e_out], INF = no path

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


  // ── PH-ALT-TG: Transition graph with ALT heuristic ──────────────────────
  //
  // Built from precomputed intra-room transfers and room boundary crossings.
  // Nodes = portal surface indices (one per room-side of each edge).
  // Edges = intra-room transfers (geometric path lengths) + boundary crossings.
  // ALT landmarks provide graph-distance lower bounds for A* on G_T.

  struct TGNode {
    int surf_idx;     ///< surface graph index of this portal
    int room_id;      ///< which room this portal is in
    int edge_idx;     ///< which room edge this portal belongs to
  };

  /// Per-edge portal TG node mapping (for multi-portal crossing edges).
  struct EdgePortals {
    std::vector<int> nodes_i;  ///< TG node IDs for room_i side
    std::vector<int> nodes_j;  ///< TG node IDs for room_j side
  };

  int tg_num_nodes_ = 0;
  std::vector<TGNode> tg_nodes_;
  std::unordered_map<int, std::vector<int>> tg_room_portals_;  ///< room_id → [node_ids]
  std::vector<EdgePortals> tg_edge_portals_;                   ///< edge_idx → portal nodes
  std::vector<std::vector<std::pair<int, double>>> tg_adj_;    ///< node_id → [(nbr, cost)]

  int tg_num_landmarks_ = 0;
  std::vector<double> tg_landmark_dist_;   ///< [l * tg_num_nodes_ + v]

  bool use_ph_alt_ = false;

  /// Build transition graph G_T from precomputed transfers.  Called by setTopoMap().
  void buildTransitionGraph();

  /// A* on transition graph with ALT heuristic.  Returns RoomRoute
  /// compatible with stitchPath().  Populates diag if non-null.
  RoomRoute transitionGraphAstar(
    int start_surf, int goal_surf,
    const std::tuple<double, double, double> & start_world,
    const std::tuple<double, double, double> & goal_world,
    QueryDiagnostics * diag = nullptr) const;

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

  // ── Precompute stats ───────────────────────────────────────────────────────
  PrecomputeStats precompute_stats_;

  /// Count connected components of the transition graph via BFS.
  int countTGComponents() const;

  // ── State ─────────────────────────────────────────────────────────────────
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap_;
  double w_up_, w_down_, w_obs_;

  RoomGraph  rg_;
  FloorGraph fg_;
  bool has_topo_map_ = false;
  bool use_stitch_   = true;   ///< false = no_stitch ablation mode
};

}  // namespace topo_nav
