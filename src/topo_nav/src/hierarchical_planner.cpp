#include "topo_nav/hierarchical_planner.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_set>
#include <vector>

namespace topo_nav
{

// ─────────────────────────────────────────────────────────────────────────────
HierarchicalPlanner::HierarchicalPlanner(
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap,
  double w_up, double w_down, double w_obs)
: vmap_(std::move(vmap)), w_up_(w_up), w_down_(w_down), w_obs_(w_obs)
{}

void HierarchicalPlanner::setTopoMap(const RoomGraph & rg, const FloorGraph & fg)
{
  rg_ = rg;
  fg_ = fg;
  has_topo_map_ = true;

  // ── Initialise persistent dirty-array A* state ────────────────────────────
  // Allocate exactly once; reuse across all voxelAstar() calls via generation
  // counter.  astar_gen_ = 0 means "no call yet"; first call bumps to 1.
  const int n = static_cast<int>(vmap_->getSurfaceGraph().size());
  astar_g_.resize(n);
  astar_came_from_.resize(n);
  astar_g_gen_.assign(n, 0u);   // 0 = uninitialised (gen counter starts at 1)
  astar_cl_gen_.assign(n, 0u);
  astar_gen_ = 0u;

  // ── Precompute HPA* transfer table ────────────────────────────────────────
  transfer_E_          = 0;
  transfer_cost_.clear();
  transfer_path_.clear();
  min_exit_cost_flat_.clear();
  precomputeTransfers();
}

// ─────────────────────────────────────────────────────────────────────────────
// HPA* precomputation: intra-room transition-to-transition voxel paths
//
// For each room r with k incident edges: runs k*(k-1) voxelAstar calls
// restricted to that room, storing the results in flat arrays indexed by
// (edge_in * E + edge_out).
//
// Thanks to the dirty-array trick, each call pays O(|room_voxels|) not O(n).
// Typical precompute time: 18 rooms × 6 transfers × ~2 ms = ~110 ms.
// ─────────────────────────────────────────────────────────────────────────────
void HierarchicalPlanner::precomputeTransfers()
{
  const int E = static_cast<int>(rg_.edges.size());
  const int R = rg_.num_rooms;
  transfer_E_ = E;

  if (E == 0 || R == 0) return;

  const double INF = std::numeric_limits<double>::infinity();

  // Flat tables: E×E for transfer, R×E for min exit cost
  transfer_cost_.assign(static_cast<size_t>(E) * E, INF);
  transfer_path_.resize(static_cast<size_t>(E) * E);  // default empty vectors
  min_exit_cost_flat_.assign(static_cast<size_t>(R) * E, INF);

  for (int r = 0; r < R; ++r) {
    const auto adj_it = rg_.adjacency.find(r);
    if (adj_it == rg_.adjacency.end()) continue;

    // Collect (edge_idx, transition_surf on room-r side) for all incident edges
    std::vector<std::pair<int, int>> incident;
    incident.reserve(adj_it->second.size());
    for (auto & [nbr, eidx] : adj_it->second) {
      const auto & edge = rg_.edges[eidx];
      const int t = (edge.room_i == r) ? edge.transition_surf_i
                                       : edge.transition_surf_j;
      incident.emplace_back(eidx, t);
    }
    if (incident.size() < 2) continue;  // dead-end: no through-paths to precompute

    for (size_t ii = 0; ii < incident.size(); ++ii) {
      for (size_t jj = 0; jj < incident.size(); ++jj) {
        if (ii == jj) continue;
        const auto [ei, t_i] = incident[ii];
        const auto [ej, t_j] = incident[jj];

        auto path = voxelAstar(t_i, t_j, r);
        const double cost = path.empty() ? INF : computePathLength(path);

        const int slot = ei * E + ej;
        transfer_cost_[slot] = cost;
        if (!path.empty()) transfer_path_[slot] = std::move(path);

        // Track minimum exit cost for (room r, exit edge ej)
        double & mc = min_exit_cost_flat_[r * E + ej];
        if (cost < mc) mc = cost;
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Snap world coordinate to surface graph index
// ─────────────────────────────────────────────────────────────────────────────
int HierarchicalPlanner::snapToSurface(
  const std::tuple<double, double, double> & world,
  bool snap) const
{
  auto [wx, wy, wz] = world;
  auto [ix, iy, iz] = vmap_->worldToGrid(wx, wy, wz);
  if (!vmap_->isValidIdx(ix, iy, iz)) return -1;

  int idx = vmap_->coordToSurfaceIdx(ix, iy, iz);
  if (idx >= 0) return idx;
  if (!snap) return -1;

  const int radius = static_cast<int>(5.0 / vmap_->getResolution());
  auto [nx, ny, nz] = vmap_->findNearestValidSurface(ix, iy, iz, radius);
  if (nx < 0) return -1;
  return vmap_->coordToSurfaceIdx(nx, ny, nz);
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute path length in metres
// ─────────────────────────────────────────────────────────────────────────────
double HierarchicalPlanner::computePathLength(
  const std::vector<int> & surf_indices) const
{
  const auto & graph = vmap_->getSurfaceGraph();
  const double res   = vmap_->getResolution();
  double len = 0.0;
  for (size_t i = 1; i < surf_indices.size(); ++i) {
    const auto & a = graph[surf_indices[i - 1]];
    const auto & b = graph[surf_indices[i]];
    const double dx = (b.x - a.x) * res;
    const double dy = (b.y - a.y) * res;
    const double dz = (b.z - a.z) * res;
    len += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return len;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 1: Floor-level A*
// ─────────────────────────────────────────────────────────────────────────────
std::vector<int> HierarchicalPlanner::floorAstar(
  int start_floor, int goal_floor) const
{
  if (start_floor == goal_floor) return {start_floor};

  const int F = static_cast<int>(fg_.floors.size());
  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> g(F, INF);
  std::vector<int>    came_from(F, -1);
  std::vector<bool>   closed(F, false);

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  g[start_floor] = 0.0;
  open.emplace(0.0, start_floor);

  auto h = [&](int f) {
    return std::abs(f - goal_floor) * 3.0;
  };

  while (!open.empty()) {
    auto [_, cur] = open.top(); open.pop();
    if (closed[cur]) continue;
    if (cur == goal_floor) {
      std::vector<int> path;
      for (int f = cur; f >= 0; f = came_from[f]) {
        path.push_back(f);
        if (f == start_floor) break;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    closed[cur] = true;

    if (static_cast<size_t>(cur) >= fg_.adjacency.size()) continue;
    for (auto & [nbr, eidx] : fg_.adjacency[cur]) {
      if (closed[nbr]) continue;
      double cost = g[cur] + fg_.edges[eidx].traversal_cost;
      if (cost < g[nbr]) {
        g[nbr] = cost;
        came_from[nbr] = cur;
        open.emplace(cost + h(nbr), nbr);
      }
    }
  }
  return {};
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 2: Room-level A*
// ─────────────────────────────────────────────────────────────────────────────
std::vector<int> HierarchicalPlanner::roomAstar(
  int start_room,
  int goal_room,
  const std::vector<int> & allowed_floors,
  float robot_diameter_m) const
{
  if (start_room == goal_room) return {start_room};

  const int R = rg_.num_rooms;
  const int E = transfer_E_;
  const double INF = std::numeric_limits<double>::infinity();

  std::unordered_set<int> floor_set(allowed_floors.begin(), allowed_floors.end());
  std::vector<bool> allowed(R, false);
  for (int r = 0; r < R; ++r) {
    if (floor_set.count(rg_.rooms[r].floor_id)) allowed[r] = true;
    if (rg_.rooms[r].type == RoomType::STAIRCASE) allowed[r] = true;
  }
  allowed[start_room] = true;
  allowed[goal_room]  = true;

  std::vector<double> g(R, INF);
  std::vector<int>    came_from(R, -1);
  std::vector<bool>   closed(R, false);

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  g[start_room] = 0.0;
  open.emplace(0.0, start_room);

  const auto & gr = rg_.rooms[goal_room];
  auto h = [&](int r) -> double {
    const auto & rm = rg_.rooms[r];
    const double dx = rm.cx - gr.cx;
    const double dy = rm.cy - gr.cy;
    const double dz = rm.cz - gr.cz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  while (!open.empty()) {
    auto [_, cur] = open.top(); open.pop();
    if (closed[cur]) continue;
    if (cur == goal_room) {
      std::vector<int> path;
      for (int r = cur; r >= 0; r = came_from[r]) {
        path.push_back(r);
        if (r == start_room) break;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    closed[cur] = true;

    const auto it = rg_.adjacency.find(cur);
    if (it == rg_.adjacency.end()) continue;

    for (auto & [nbr, eidx] : it->second) {
      if (!allowed[nbr] || closed[nbr]) continue;
      const auto & edge = rg_.edges[eidx];
      if (edge.passage_width_m < robot_diameter_m) continue;

      const auto & rm_cur = rg_.rooms[cur];
      const auto & rm_nbr = rg_.rooms[nbr];

      // Within-room traversal cost: flat lookup O(1).
      // min_exit_cost_flat_[cur * E + eidx] = minimum actual voxel path cost
      // to traverse room `cur` and exit via edge `eidx`, over all entry edges.
      // Falls back to Euclidean centroid→transition if not precomputed.
      double within_cost;
      if (E > 0) {
        const double mc = min_exit_cost_flat_[cur * E + eidx];
        if (mc < INF) {
          within_cost = mc;
        } else if (mc >= INF) {
          // INF means either "no valid path" (precomputed) or "not in table"
          // (dead-end room with single edge).  Distinguish by checking if
          // room `cur` has >= 2 incident edges.
          const auto it2 = rg_.adjacency.find(cur);
          const bool has_transfer =
            (it2 != rg_.adjacency.end() && it2->second.size() >= 2);
          if (has_transfer) {
            continue;  // genuinely no path through this exit — skip edge
          }
          // Single-edge room: Euclidean fallback
          const double dtx = edge.trans_wx - rm_cur.cx;
          const double dty = edge.trans_wy - rm_cur.cy;
          const double dtz = edge.trans_wz - rm_cur.cz;
          within_cost = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
        }
      } else {
        // No precomputed table (build_topo_map not yet called?): Euclidean
        const double dtx = edge.trans_wx - rm_cur.cx;
        const double dty = edge.trans_wy - rm_cur.cy;
        const double dtz = edge.trans_wz - rm_cur.cz;
        within_cost = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
      }

      // Cost from transition to neighbour centroid (Euclidean approximation)
      const double dtx2 = rm_nbr.cx - edge.trans_wx;
      const double dty2 = rm_nbr.cy - edge.trans_wy;
      const double dtz2 = rm_nbr.cz - edge.trans_wz;
      const double d2   = std::sqrt(dtx2 * dtx2 + dty2 * dty2 + dtz2 * dtz2);

      const double cost = g[cur] + within_cost + d2;
      if (cost < g[nbr]) {
        g[nbr]        = cost;
        came_from[nbr] = cur;
        open.emplace(cost + h(nbr), nbr);
      }
    }
  }
  return {};
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 3: Voxel-level A* (room-restricted or unrestricted)
//
// Performance: uses the dirty-array trick to avoid O(n) initialisation per
// call.  Only cells actually touched get stamped; bumping astar_gen_ (O(1))
// "resets" all cells.  For room-restricted search with |room| << n (e.g.
// 2 000 out of 50 000 surface voxels), this removes a ~325× overhead ratio.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<int> HierarchicalPlanner::voxelAstar(
  int start_surf, int goal_surf, int allowed_room_id) const
{
  if (start_surf == goal_surf) return {start_surf};

  const auto & graph    = vmap_->getSurfaceGraph();
  const auto & all_nbrs = vmap_->getAllNeighbors();
  const double res = vmap_->getResolution();
  const double INF = std::numeric_limits<double>::infinity();

  // ── Dirty-array "reset": O(1) instead of O(n) ────────────────────────────
  ++astar_gen_;
  if (astar_gen_ == 0u) {
    // Counter wrapped (after 4 billion calls): hard-reset generation stamps.
    std::fill(astar_g_gen_.begin(),  astar_g_gen_.end(),  0u);
    std::fill(astar_cl_gen_.begin(), astar_cl_gen_.end(), 0u);
    ++astar_gen_;  // now == 1, distinct from the reset sentinel 0
  }
  const uint32_t gen = astar_gen_;

  // ── Inline accessors (optimise away in release builds) ────────────────────
  // g[i]:         uninitialised → INF; otherwise astar_g_[i]
  // came_from[i]: uninitialised → -1;  otherwise astar_came_from_[i]
  // is_closed[i]: astar_cl_gen_[i] == gen

  // ── Heuristic ─────────────────────────────────────────────────────────────
  const auto & gn = graph[goal_surf];
  auto h = [&](int idx) -> double {
    const auto & nd = graph[idx];
    const double dx = (nd.x - gn.x) * res;
    const double dy = (nd.y - gn.y) * res;
    const double dz = (nd.z - gn.z) * res;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  // ── Edge cost (same as JPS3D) ─────────────────────────────────────────────
  auto edge_cost = [&](int from_idx, int to_idx) -> double {
    const auto & nf = graph[from_idx];
    const auto & nt = graph[to_idx];
    const int dx = std::abs(nt.x - nf.x);
    const int dy = std::abs(nt.y - nf.y);
    const int dz_signed = nt.z - nf.z;
    const int dz = std::abs(dz_signed);
    const double eucl = res * std::sqrt(
      static_cast<double>(dx*dx + dy*dy + dz*dz));
    const double wz = (dz_signed > 0) ? w_up_ :
                      (dz_signed < 0) ? w_down_ : 0.0;
    const double z_cost = dz * res * wz;
    const double clear  = w_obs_ * res / (nt.distance + 1);
    return eucl + z_cost + clear;
  };

  // ── Open list ─────────────────────────────────────────────────────────────
  using QElem = std::tuple<double, double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  // Initialise start node (dirty write)
  astar_g_gen_[start_surf]    = gen;
  astar_g_[start_surf]        = 0.0;
  astar_came_from_[start_surf] = -1;
  open.emplace(h(start_surf), 0.0, start_surf);

  while (!open.empty()) {
    auto [_, cur_g, cur] = open.top(); open.pop();

    // Skip stale entries (lazy deletion)
    if (astar_cl_gen_[cur] == gen) continue;

    if (cur == goal_surf) {
      // ── Reconstruct path ─────────────────────────────────────────────────
      std::vector<int> path;
      int nd = goal_surf;
      while (nd >= 0) {
        path.push_back(nd);
        if (nd == start_surf) break;
        nd = (astar_g_gen_[nd] == gen) ? astar_came_from_[nd] : -1;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Mark closed (dirty write)
    astar_cl_gen_[cur] = gen;

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];

      if (astar_cl_gen_[nbr] == gen) continue;

      // Room filter
      if (allowed_room_id >= 0 &&
          nbr != goal_surf &&
          rg_.room_labels[nbr] != allowed_room_id)
      {
        continue;
      }

      const double tentative_g = cur_g + edge_cost(cur, nbr);
      const double nbr_g = (astar_g_gen_[nbr] == gen) ? astar_g_[nbr] : INF;

      if (tentative_g < nbr_g) {
        astar_g_gen_[nbr]     = gen;
        astar_g_[nbr]         = tentative_g;
        astar_came_from_[nbr] = cur;
        open.emplace(tentative_g + h(nbr), tentative_g, nbr);
      }
    }
  }
  return {};
}

// ─────────────────────────────────────────────────────────────────────────────
// Main entry: three-level hierarchical planning
// ─────────────────────────────────────────────────────────────────────────────
HierarchicalPath HierarchicalPlanner::plan(
  const std::tuple<double, double, double> & start_world,
  const std::tuple<double, double, double> & goal_world,
  float robot_diameter_m,
  bool snap_to_surface)
{
  HierarchicalPath result;
  auto total_t0 = std::chrono::steady_clock::now();

  if (!has_topo_map_) {
    result.message = "Topo map not built. Call BuildTopoMap first.";
    return result;
  }

  // ── Snap start / goal to surface ─────────────────────────────────────────
  const int start_surf = snapToSurface(start_world, snap_to_surface);
  const int goal_surf  = snapToSurface(goal_world,  snap_to_surface);
  if (start_surf < 0 || goal_surf < 0) {
    result.message = "Cannot snap start/goal to surface.";
    return result;
  }

  const int start_room  = rg_.room_labels[start_surf];
  const int goal_room   = rg_.room_labels[goal_surf];
  const int start_floor = (start_room >= 0) ? rg_.rooms[start_room].floor_id : -1;
  const int goal_floor  = (goal_room  >= 0) ? rg_.rooms[goal_room].floor_id  : -1;

  if (start_room < 0 || goal_room < 0 || start_floor < 0 || goal_floor < 0) {
    result.message = "Start or goal not assigned to a room/floor.";
    return result;
  }

  // ── Step 1: Floor-level routing (flat mode: allow all floors) ────────────
  auto floor_t0 = std::chrono::steady_clock::now();
  // Build all-floors list so roomAstar can traverse the full room graph
  // without being constrained by floor boundaries. This is equivalent to
  // treating the entire building as a single flat topology (room + corridor).
  std::vector<int> floor_seq;
  const int n_floors = static_cast<int>(fg_.floors.size());
  floor_seq.reserve(n_floors > 0 ? n_floors : 1);
  for (int f = 0; f < n_floors; ++f) floor_seq.push_back(f);
  if (floor_seq.empty()) floor_seq.push_back(0);  // fallback: single floor
  auto floor_t1 = std::chrono::steady_clock::now();
  result.floor_planning_ms =
    std::chrono::duration<double, std::milli>(floor_t1 - floor_t0).count();
  result.floor_sequence = floor_seq;

  // ── Step 2: Room-level routing ────────────────────────────────────────────
  auto room_t0 = std::chrono::steady_clock::now();
  const std::vector<int> room_seq =
    roomAstar(start_room, goal_room, floor_seq, robot_diameter_m);
  auto room_t1 = std::chrono::steady_clock::now();
  result.room_planning_ms =
    std::chrono::duration<double, std::milli>(room_t1 - room_t0).count();

  if (room_seq.empty()) {
    // Debug: print adjacency sizes for start/goal rooms
    RCLCPP_WARN(rclcpp::get_logger("hierarchical_planner"),
      "Room routing failed: start_room=%d (adj=%zu, floor=%d), "
      "goal_room=%d (adj=%zu, floor=%d), n_rooms=%d, n_floors=%d, robot_d=%.3f",
      start_room,
      start_room >= 0 ? rg_.adjacency[start_room].size() : 0,
      start_room >= 0 ? rg_.rooms[start_room].floor_id : -1,
      goal_room,
      goal_room >= 0 ? rg_.adjacency[goal_room].size() : 0,
      goal_room >= 0 ? rg_.rooms[goal_room].floor_id : -1,
      (int)rg_.rooms.size(), (int)fg_.floors.size(), robot_diameter_m);
    result.message = "Room-level routing failed (no passable path?).";
    return result;
  }
  result.room_sequence = room_seq;

  // ── Step 3: Voxel-level refinement ────────────────────────────────────────
  auto voxel_t0 = std::chrono::steady_clock::now();

  const int E = transfer_E_;
  const int P = static_cast<int>(room_seq.size());
  std::vector<int> all_surf_path;
  all_surf_path.reserve(512);
  all_surf_path.push_back(start_surf);

  int prev_entry    = start_surf;
  int total_expanded = 0;

  for (int k = 0; k < P; ++k) {
    const int cur_room = room_seq[k];

    // ── Find exit edge / surface for this room ────────────────────────────
    int exit_eidx = -1;
    int exit_surf;
    if (k < P - 1) {
      const int next_room = room_seq[k + 1];
      const auto it = rg_.adjacency.find(cur_room);
      if (it != rg_.adjacency.end()) {
        for (auto & [nbr, ei] : it->second) {
          if (nbr == next_room) { exit_eidx = ei; break; }
        }
      }
      if (exit_eidx < 0) {
        result.message = "No edge between consecutive rooms in sequence.";
        return result;
      }
      const auto & edge = rg_.edges[exit_eidx];
      exit_surf = (edge.room_i == cur_room) ? edge.transition_surf_i
                                            : edge.transition_surf_j;
    } else {
      exit_surf = goal_surf;
    }

    // ── Build voxel segment for this room ─────────────────────────────────
    std::vector<int> seg;

    // Middle rooms: flat O(1) lookup into precomputed transfer table
    if (k > 0 && k < P - 1 && E > 0) {
      const int prev_room = room_seq[k - 1];
      int entry_eidx = -1;
      const auto it = rg_.adjacency.find(cur_room);
      if (it != rg_.adjacency.end()) {
        for (auto & [nbr, ei] : it->second) {
          if (nbr == prev_room) { entry_eidx = ei; break; }
        }
      }
      if (entry_eidx >= 0 && exit_eidx >= 0) {
        const auto & pre = transfer_path_[entry_eidx * E + exit_eidx];
        if (!pre.empty()) {
          seg = pre;  // copy precomputed path
        }
      }
    }

    // First / last room (or missing precomputed path): runtime A*
    if (seg.empty()) {
      seg = voxelAstar(prev_entry, exit_surf, cur_room);
      if (seg.empty()) {
        seg = voxelAstar(prev_entry, exit_surf, -1);
      }
    }

    if (seg.empty()) {
      result.message = "Voxel-level A* failed in room " + std::to_string(cur_room);
      return result;
    }

    // ── Append segment (skip first element if it duplicates tail) ─────────
    const size_t start_si = (seg.front() == all_surf_path.back()) ? 1u : 0u;
    all_surf_path.insert(all_surf_path.end(),
                         seg.begin() + start_si, seg.end());
    total_expanded += static_cast<int>(seg.size());

    // ── Set up entry for next room ────────────────────────────────────────
    if (k < P - 1) {
      const int next_room = room_seq[k + 1];
      const auto & graph    = vmap_->getSurfaceGraph();
      const auto & all_nbrs = vmap_->getAllNeighbors();
      const auto & nd = graph[exit_surf];

      int entry_next = -1;
      int best_d = -1;
      for (int ni = 0; ni < nd.neighbor_count; ++ni) {
        const int q = all_nbrs[nd.neighbor_start + ni];
        if (rg_.room_labels[q] == next_room && graph[q].distance > best_d) {
          best_d     = graph[q].distance;
          entry_next = q;
        }
      }
      if (entry_next < 0 && exit_eidx >= 0) {
        // Fallback: canonical transition surface on next room's side
        const auto & edge = rg_.edges[exit_eidx];
        entry_next = (edge.room_i == next_room) ? edge.transition_surf_i
                                                 : edge.transition_surf_j;
      }
      if (entry_next < 0) {
        result.message = "Cannot find entry into room " + std::to_string(next_room);
        return result;
      }

      if (entry_next != all_surf_path.back()) {
        all_surf_path.push_back(entry_next);
      }
      prev_entry = entry_next;
    }
  }

  auto voxel_t1 = std::chrono::steady_clock::now();
  result.voxel_planning_ms =
    std::chrono::duration<double, std::milli>(voxel_t1 - voxel_t0).count();
  result.voxel_nodes_expanded = total_expanded;

  // ── Convert surface-graph indices to world coordinates ────────────────────
  const auto & graph = vmap_->getSurfaceGraph();
  result.waypoints.reserve(all_surf_path.size());
  for (int idx : all_surf_path) {
    const auto & nd = graph[idx];
    result.waypoints.push_back(vmap_->gridToWorld(nd.x, nd.y, nd.z));
  }

  result.path_length_m = computePathLength(all_surf_path);

  auto total_t1 = std::chrono::steady_clock::now();
  result.total_planning_ms =
    std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

  result.success = true;
  result.message = "OK";
  return result;
}

}  // namespace topo_nav
