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
  transfer_length_.assign(static_cast<size_t>(E) * E, INF);
  transfer_path_.resize(static_cast<size_t>(E) * E);  // default empty vectors
  min_exit_cost_flat_.assign(static_cast<size_t>(R) * E, INF);

  trans_field_.clear();

  // Precompute per-room voxel counts once (used for budget cap + Dijkstra termination)
  std::vector<int> room_voxel_count(R, 0);
  {
    const auto & graph_count = vmap_->getSurfaceGraph();
    for (int vi = 0; vi < static_cast<int>(graph_count.size()); ++vi) {
      const int rv = rg_.room_labels[vi];
      if (rv >= 0 && rv < R) ++room_voxel_count[rv];
    }
  }

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

    // Budget-based cap: skip rooms whose total work exceeds threshold.
    // Work ≈ K Dijkstra calls × halo voxel count.  K=57 × 7636 = 435K still fits
    // a 500K budget (~50 ms for one big room); truly huge rooms (> 500K) are
    // still skipped to keep setTopoMap time bounded.
    static constexpr size_t kPrecomputeBudget = 1000000;
    const int room_voxels = room_voxel_count[r];
    const size_t work = incident.size() * static_cast<size_t>(room_voxels);
    if (work > kPrecomputeBudget) {
      fprintf(stderr,
        "[HPA*] room %3d: %4d voxels, %zu edges, work=%zu — skipping "
        "(> budget %zu); stitchPath will use lazy online Dijkstra or fallback\n",
        r, room_voxels, incident.size(), work, kPrecomputeBudget);
      continue;
    }

    const auto r_t0 = std::chrono::steady_clock::now();

    // Unrestricted Dijkstra with early termination at full room coverage.
    // See roomDijkstra() for rationale: transitions sit on distance-field
    // saddles and nearly always have zero same-room neighbours.
    for (size_t ii = 0; ii < incident.size(); ++ii) {
      const auto [ei, t_i] = incident[ii];

      TransitionField field;
      roomDijkstra(t_i, r, room_voxels, field.dist, field.parent);

      // Extract transfer costs to all other incident transitions
      for (size_t jj = 0; jj < incident.size(); ++jj) {
        if (ii == jj) continue;
        const auto [ej, t_j] = incident[jj];

        auto dit = field.dist.find(t_j);
        const double cost = (dit != field.dist.end()) ? dit->second : INF;

        const int slot = ei * E + ej;
        transfer_cost_[slot] = cost;

        // Reconstruct path from parent chain
        if (cost < INF) {
          std::vector<int> path;
          int cur = t_j;
          while (cur >= 0) {
            path.push_back(cur);
            if (cur == t_i) break;
            auto pit = field.parent.find(cur);
            cur = (pit != field.parent.end()) ? pit->second : -1;
          }
          std::reverse(path.begin(), path.end());
          transfer_length_[slot] = computePathLength(path);
          transfer_path_[slot] = std::move(path);
        }

        // Track minimum exit cost for (room r, exit edge ej)
        double & mc = min_exit_cost_flat_[r * E + ej];
        if (cost < mc) mc = cost;
      }

      // Store distance field for query-time O(1) lookup
      trans_field_[t_i] = std::move(field);
    }

    const auto r_t1 = std::chrono::steady_clock::now();
    const double r_ms = std::chrono::duration<double, std::milli>(r_t1 - r_t0).count();
    if (r_ms > 5.0) {
      fprintf(stderr,
        "[HPA*] room %3d: %4d voxels, %zu edges, %zu Dijkstra calls, %.1f ms\n",
        r, room_voxels, incident.size(), incident.size(), r_ms);
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
// Step 2: Room-level A* with HPA*-accurate edge costs
//
// State = (entry_edge_idx, room_id), encoded as entry_edge * R + room_id.
// "I arrived in room_id by crossing entry_edge_idx from the other side."
//
// Within-room traversal cost: transfer_cost_[entry_ei * E + exit_ej]
//   — the exact precomputed voxel path cost through room_id from entry_ei's
//     transition to exit_ej's transition.
//
// This is the correct HPA* formulation: the cost of traversing a room depends
// on BOTH the entry edge and the exit edge, not just the exit edge.
// The previous approach (min_exit_cost_flat_) discarded entry direction and
// could produce suboptimal room sequences.
//
// State space: E × R ≤ 10 000 for typical indoor scenes.  Much smaller than
// the voxel graph, so memory and runtime are negligible.
// ─────────────────────────────────────────────────────────────────────────────
HierarchicalPlanner::RoomRoute HierarchicalPlanner::roomAstar(
  int start_room,
  int goal_room,
  int start_surf,
  int goal_surf,
  const std::tuple<double, double, double> & start_world,
  const std::tuple<double, double, double> & goal_world,
  const std::vector<int> & allowed_floors,
  float /*robot_diameter_m*/) const
{
  RoomRoute route;
  if (start_room == goal_room) {
    route.rooms = {start_room};
    return route;
  }

  const int R = rg_.num_rooms;
  const int E = transfer_E_;
  if (E == 0 || R == 0) return route;

  const double INF = std::numeric_limits<double>::infinity();

  std::unordered_set<int> floor_set(allowed_floors.begin(), allowed_floors.end());
  auto room_allowed = [&](int r) -> bool {
    if (r == start_room || r == goal_room) return true;
    if (rg_.rooms[r].type == RoomType::STAIRCASE) return true;
    return (bool)floor_set.count(rg_.rooms[r].floor_id);
  };

  // Helper: room-side transition surf for (room, edge) pair
  auto roomSideTrans = [&](int room, int eidx) -> int {
    const auto & e = rg_.edges[eidx];
    return (e.room_i == room) ? e.transition_surf_i : e.transition_surf_j;
  };

  // Helper: Euclidean cost estimates for start/goal segments.
  // We intentionally use Euclidean (not precomputed penalty-weighted costs from
  // trans_field_) so that roomAstar optimises for geometric path length rather
  // than the obstacle-clearance-penalised cost function.  The penalty-weighted
  // costs would cause A* to prefer "safe" but geometrically much longer routes.
  const auto [sx, sy, sz] = start_world;
  const auto [gx_w, gy_w, gz_w] = goal_world;
  auto exactStartCost = [&](int eidx) -> double {
    const auto & edge = rg_.edges[eidx];
    const double dtx = edge.trans_wx - sx;
    const double dty = edge.trans_wy - sy;
    const double dtz = edge.trans_wz - sz;
    return std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
  };
  auto exactGoalCost = [&](int eidx) -> double {
    const auto & edge = rg_.edges[eidx];
    const double dtx = edge.trans_wx - gx_w;
    const double dty = edge.trans_wy - gy_w;
    const double dtz = edge.trans_wz - gz_w;
    return std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
  };

  // State: entry_edge_idx * R + room_id  (came_from value -1 = virtual start)
  const int STATE_COUNT = E * R;
  std::vector<double> g(STATE_COUNT, INF);
  std::vector<int>    came_from(STATE_COUNT, -2);  // -2 = not reached
  std::vector<bool>   closed(STATE_COUNT, false);

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  // Heuristic: Euclidean from edge transition to the actual goal world position.
  auto h = [&](int ei) -> double {
    const auto & edge = rg_.edges[ei];
    const double dx = edge.trans_wx - gx_w;
    const double dy = edge.trans_wy - gy_w;
    const double dz = edge.trans_wz - gz_w;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  // ── Initialise: from start_room push all its exit edges ──────────────────
  // Cost = exact geodesic from start_surf to room-side transition (when
  // trans_field is available), else Euclidean.
  const auto adj_start = rg_.adjacency.find(start_room);
  if (adj_start == rg_.adjacency.end()) return route;

  for (auto & [nbr, eidx] : adj_start->second) {
    if (!room_allowed(nbr)) continue;
    const double init_cost = exactStartCost(eidx);

    // Goal-room shortcut: if neighbour IS goal_room, add the exact end cost too.
    double admit = init_cost;
    if (nbr == goal_room) admit += exactGoalCost(eidx);

    const int state = eidx * R + nbr;
    if (admit < g[state]) {
      g[state]          = admit;
      came_from[state]  = -1;  // came from virtual start
      // For non-goal we use Euclidean h; for goal we use 0 (we already added
      // exact remaining cost).
      const double f = (nbr == goal_room) ? admit : (admit + h(eidx));
      open.emplace(f, state);
    }
  }

  // ── A* main loop ─────────────────────────────────────────────────────────
  while (!open.empty()) {
    auto [_, cur_state] = open.top(); open.pop();
    if (closed[cur_state]) continue;
    closed[cur_state] = true;

    const int entry_ei = cur_state / R;
    const int cur_room = cur_state % R;

    // Goal check
    if (cur_room == goal_room) {
      // Reconstruct (room_seq, edge_seq) by following came_from chain.
      // State (entry_ei, room) means "in room, entered via entry_ei".
      // Walk back: collect rooms; the edge BETWEEN room_seq[i] and room_seq[i+1]
      // is the entry_ei of room_seq[i+1].
      std::vector<int> rooms_rev;
      std::vector<int> edges_rev;  // edges_rev[i] is the entry_ei of rooms_rev[i]
      int s = cur_state;
      while (s >= 0) {
        rooms_rev.push_back(s % R);
        edges_rev.push_back(s / R);
        s = came_from[s];
      }
      rooms_rev.push_back(start_room);
      // Note: edges_rev is one shorter than rooms_rev once we add start_room.
      // edges_rev[i] = entry edge of rooms_rev[i] for i in [0, rooms_rev.size()-2]

      route.rooms.assign(rooms_rev.rbegin(), rooms_rev.rend());
      route.edges.assign(edges_rev.rbegin(), edges_rev.rend());
      // route.edges[i] is the edge between rooms[i] and rooms[i+1].

      fprintf(stderr, "[roomAstar] seq(%zu rooms, %zu edges): ",
              route.rooms.size(), route.edges.size());
      for (int ri : route.rooms) fprintf(stderr, "%d ", ri);
      fprintf(stderr, "\n");
      return route;
    }

    // ── Expand: traverse cur_room from entry_ei to each exit edge ────────
    const auto adj_it = rg_.adjacency.find(cur_room);
    if (adj_it == rg_.adjacency.end()) continue;

    for (auto & [nbr, ejdx] : adj_it->second) {
      if (ejdx == entry_ei) continue;   // no immediate back-tracking
      if (!room_allowed(nbr)) continue;

      // Precomputed geometric path length through cur_room (Euclidean-only
      // Dijkstra paths).  Falls back to straight-line Euclidean when no
      // precomputed path exists (room skipped by budget cap).
      double within_cost = transfer_length_[entry_ei * E + ejdx];
      if (within_cost >= INF) {
        const auto & e_in  = rg_.edges[entry_ei];
        const auto & e_out = rg_.edges[ejdx];
        const double dx = e_out.trans_wx - e_in.trans_wx;
        const double dy = e_out.trans_wy - e_in.trans_wy;
        const double dz = e_out.trans_wz - e_in.trans_wz;
        within_cost = std::sqrt(dx * dx + dy * dy + dz * dz);
      }

      double new_g = g[cur_state] + within_cost;
      // Goal-room admission: add exact transition→goal_surf cost.
      if (nbr == goal_room) new_g += exactGoalCost(ejdx);

      const int next_state = ejdx * R + nbr;
      if (new_g < g[next_state]) {
        g[next_state]         = new_g;
        came_from[next_state] = cur_state;
        const double f = (nbr == goal_room) ? new_g : (new_g + h(ejdx));
        open.emplace(f, next_state);
      }
    }
  }
  return route;
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
  int start_surf, int goal_surf, int allowed_room_id, int* out_expanded,
  const std::unordered_set<int>* corridor) const
{
  if (out_expanded) *out_expanded = 0;
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
    if (out_expanded) ++(*out_expanded);

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];

      if (astar_cl_gen_[nbr] == gen) continue;

      // Room filter: corridor set takes priority over single-room filter
      if (corridor) {
        if (nbr != goal_surf && !corridor->count(rg_.room_labels[nbr]))
          continue;
      } else if (allowed_room_id >= 0 &&
                 nbr != goal_surf &&
                 rg_.room_labels[nbr] != allowed_room_id) {
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
// Scheme 2: Exhaustive single-source Dijkstra restricted to one room.
//
// Runs Dijkstra (no heuristic) from source_surf, visiting ALL reachable
// voxels in room_id.  This produces a complete shortest-path tree from
// the source transition to every room voxel.
//
// Called once per transition during precomputeTransfers(), replacing the
// previous k*(k-1) pairwise A* calls with k single-source Dijkstra calls.
// Also stores distance fields for O(1) query-time cost lookup.
// ─────────────────────────────────────────────────────────────────────────────
void HierarchicalPlanner::roomDijkstra(
  int source_surf, int room_id, int room_voxel_count,
  std::unordered_map<int, double> & out_dist,
  std::unordered_map<int, int>    & out_parent) const
{
  const auto & graph    = vmap_->getSurfaceGraph();
  const auto & all_nbrs = vmap_->getAllNeighbors();
  const double res = vmap_->getResolution();

  // Dirty-array reset
  ++astar_gen_;
  if (astar_gen_ == 0u) {
    std::fill(astar_g_gen_.begin(),  astar_g_gen_.end(),  0u);
    std::fill(astar_cl_gen_.begin(), astar_cl_gen_.end(), 0u);
    ++astar_gen_;
  }
  const uint32_t gen = astar_gen_;

  // Pure Euclidean edge cost — produces geometrically shortest paths.
  // The penalty-weighted cost (z-weight + obstacle clearance) is used only
  // in voxelAstar for real-time navigation; here we want geometric
  // optimality so that precomputed paths are as short as possible.
  auto edge_cost = [&](int from_idx, int to_idx) -> double {
    const auto & nf = graph[from_idx];
    const auto & nt = graph[to_idx];
    const int dx = std::abs(nt.x - nf.x);
    const int dy = std::abs(nt.y - nf.y);
    const int dz = std::abs(nt.z - nf.z);
    return res * std::sqrt(static_cast<double>(dx*dx + dy*dy + dz*dz));
  };

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  astar_g_gen_[source_surf]     = gen;
  astar_g_[source_surf]         = 0.0;
  astar_came_from_[source_surf] = -1;
  open.emplace(0.0, source_surf);

  // Track visited nodes for efficient extraction
  std::vector<int> visited;
  visited.reserve(room_voxel_count > 0 ? room_voxel_count * 2 : 4096);

  // Unrestricted Dijkstra with early termination:
  // Persistent-homology transitions sit on distance-field saddles and
  // almost always have zero same-room neighbours.  Room-restricted
  // Dijkstra is fundamentally broken for them.  Instead we run
  // unrestricted and stop once all room-interior voxels are covered.
  int room_voxels_closed = 0;
  // Safety cap: 5× room size handles rooms with extensive boundary contact.
  const int safety_cap = std::max(room_voxel_count * 5,
                                  static_cast<int>(graph.size()));

  while (!open.empty()) {
    auto [cur_g, cur] = open.top(); open.pop();
    if (astar_cl_gen_[cur] == gen) continue;
    astar_cl_gen_[cur] = gen;
    visited.push_back(cur);

    if (rg_.room_labels[cur] == room_id) ++room_voxels_closed;
    if (room_voxels_closed >= room_voxel_count) break;
    if (static_cast<int>(visited.size()) >= safety_cap) break;

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];
      if (astar_cl_gen_[nbr] == gen) continue;

      const double tentative_g = cur_g + edge_cost(cur, nbr);
      const double nbr_g = (astar_g_gen_[nbr] == gen)
                             ? astar_g_[nbr]
                             : std::numeric_limits<double>::infinity();

      if (tentative_g < nbr_g) {
        astar_g_gen_[nbr]     = gen;
        astar_g_[nbr]         = tentative_g;
        astar_came_from_[nbr] = cur;
        open.emplace(tentative_g, nbr);
      }
    }
  }

  // Store all visited voxels: bridge voxels (transiently in adjacent rooms)
  // must be included for parent-chain walks to work in stitchPath.
  out_dist.clear();
  out_parent.clear();
  out_dist.reserve(visited.size());
  out_parent.reserve(visited.size());
  for (int v : visited) {
    out_dist[v]   = astar_g_[v];
    out_parent[v] = astar_came_from_[v];
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 3 (new): assemble the surface path from precomputed segments.
//
// Strategy: prefer O(1) lookups, fall back gracefully per segment.
//
//   Segment 0 (start_room → first transition):
//     Try trans_field_[t_exit] parent-chain walk from start_surf.
//     Fallback: voxelAstar restricted to start_room.
//
//   Intermediate (room r, entry_edge → exit_edge):
//     Try transfer_path_[e_in * E + e_out].
//     Fallback: voxelAstar restricted to room r.
//
//   Final (last transition → goal_surf in goal_room):
//     Try trans_field_[t_enter] parent-chain walk from goal_surf (reversed).
//     Fallback: voxelAstar restricted to goal_room.
//
// out_expanded counts voxel expansions from fallback segments only.
// A pure-cache hit has out_expanded == 0.
// Returns empty only if a per-segment voxelAstar fails (unreachable).
// ─────────────────────────────────────────────────────────────────────────────
std::vector<int> HierarchicalPlanner::stitchPath(
  int start_surf, int goal_surf, const RoomRoute & route,
  int * out_expanded, const std::unordered_set<int>* fallback_corridor) const
{
  if (out_expanded) *out_expanded = 0;

  fprintf(stderr, "[stitch-dbg] called: rooms=%zu edges=%zu\n",
          route.rooms.size(), route.edges.size());

  // Caller handles single-room queries (no edges to stitch).
  if (route.rooms.size() < 2 || route.edges.empty()) {
    fprintf(stderr, "[stitch-dbg] FAIL-A\n"); return {}; }
  if (route.edges.size() != route.rooms.size() - 1) {
    fprintf(stderr, "[stitch-dbg] FAIL-B\n"); return {}; }

  const int E = transfer_E_;

  auto roomSideTrans = [&](int room, int eidx) -> int {
    const auto & e = rg_.edges[eidx];
    return (e.room_i == room) ? e.transition_surf_i : e.transition_surf_j;
  };

  auto append = [](std::vector<int> & dst, const std::vector<int> & src) {
    if (src.empty()) return;
    if (!dst.empty() && dst.back() == src.front())
      dst.insert(dst.end(), src.begin() + 1, src.end());
    else
      dst.insert(dst.end(), src.begin(), src.end());
  };

  // Walk parent chain from 'from' toward 'target' in a trans_field.
  // Returns {from, ..., target} on success, empty on failure.
  auto walkToTarget = [](const TransitionField & field, int from, int target)
    -> std::vector<int>
  {
    std::vector<int> seg;
    int cur = from;
    for (int guard = 0; guard < 200000; ++guard) {
      seg.push_back(cur);
      if (cur == target) return seg;
      auto pit = field.parent.find(cur);
      if (pit == field.parent.end()) return {};
      cur = pit->second;
    }
    return {};  // exceeded guard
  };

  // Per-segment fallback counter
  int total_fallback_exp = 0;

  std::vector<int> path;
  path.reserve(256);

  // ── Segment 0: start_room → t_exit ───────────────────────────────────────
  {
    const int start_room = route.rooms[0];
    const int e0         = route.edges[0];
    const int t_exit     = roomSideTrans(start_room, e0);
    bool cached = false;

    fprintf(stderr, "[stitch-seg0] start_surf=%d(r%d) → t_exit=%d(r%d) e0=%d\n",
            start_surf, rg_.room_labels[start_surf],
            t_exit, rg_.room_labels[t_exit], e0);

    if (start_surf == t_exit) {
      path.push_back(start_surf);
      cached = true;
    } else {
      auto fit = trans_field_.find(t_exit);
      if (fit != trans_field_.end()) {
        bool has_start = fit->second.dist.count(start_surf) > 0;
        fprintf(stderr, "[stitch-seg0] trans_field_[%d] exists, |dist|=%zu, has_start=%d\n",
                t_exit, fit->second.dist.size(), has_start);
        if (has_start) {
          auto seg = walkToTarget(fit->second, start_surf, t_exit);
          if (!seg.empty()) { append(path, seg); cached = true;
            fprintf(stderr, "[stitch-seg0] CACHED %zu voxels\n", seg.size());
          } else {
            fprintf(stderr, "[stitch-seg0] walkToTarget failed\n");
          }
        }
      } else {
        fprintf(stderr, "[stitch-seg0] NO trans_field_ for t_exit=%d\n", t_exit);
      }
      if (!cached) {
        int exp = 0;
        auto seg = voxelAstar(start_surf, t_exit, -1, &exp, fallback_corridor);
        fprintf(stderr, "[stitch-seg0] FALLBACK A* → %zu voxels, exp=%d\n",
                seg.size(), exp);
        if (seg.empty()) { fprintf(stderr, "[stitch-seg0] FAIL\n"); return {}; }
        append(path, seg);
        total_fallback_exp += exp;
      }
    }
  }

  // ── Intermediate segments: transfer_path_ or per-room A* ─────────────────
  for (size_t i = 1; i + 1 < route.rooms.size(); ++i) {
    const int r       = route.rooms[i];
    const int e_in    = route.edges[i - 1];
    const int e_out   = route.edges[i];
    const int t_enter = roomSideTrans(r, e_in);
    const int t_exit  = roomSideTrans(r, e_out);

    fprintf(stderr, "[stitch-mid%zu] room=%d e_in=%d→e_out=%d t_enter=%d(r%d)→t_exit=%d(r%d)\n",
            i, r, e_in, e_out,
            t_enter, rg_.room_labels[t_enter],
            t_exit, rg_.room_labels[t_exit]);

    // Bridge across edge boundary (one extra voxel hop between rooms)
    if (path.empty() || path.back() != t_enter) path.push_back(t_enter);
    if (t_enter == t_exit) { fprintf(stderr, "[stitch-mid%zu] same voxel, skip\n", i); continue; }

    const int slot = e_in * E + e_out;
    const auto & seg_cached = transfer_path_[slot];
    if (!seg_cached.empty()) {
      append(path, seg_cached);
      fprintf(stderr, "[stitch-mid%zu] CACHED %zu voxels (slot=%d)\n",
              i, seg_cached.size(), slot);
    } else {
      fprintf(stderr, "[stitch-mid%zu] NO transfer_path_[%d], fallback A*\n", i, slot);
      int exp = 0;
      auto seg = voxelAstar(t_enter, t_exit, -1, &exp, fallback_corridor);
      fprintf(stderr, "[stitch-mid%zu] FALLBACK A* → %zu voxels, exp=%d\n",
              i, seg.size(), exp);
      if (seg.empty()) { fprintf(stderr, "[stitch-mid%zu] FAIL\n", i); return {}; }
      append(path, seg);
      total_fallback_exp += exp;
    }
  }

  // ── Final segment: t_enter → goal_surf ───────────────────────────────────
  {
    const int goal_room = route.rooms.back();
    const int e_last    = route.edges.back();
    const int t_enter   = roomSideTrans(goal_room, e_last);

    fprintf(stderr, "[stitch-fin] t_enter=%d(r%d) → goal_surf=%d(r%d) e_last=%d\n",
            t_enter, rg_.room_labels[t_enter],
            goal_surf, rg_.room_labels[goal_surf], e_last);

    if (path.empty() || path.back() != t_enter) path.push_back(t_enter);
    if (t_enter == goal_surf) { fprintf(stderr, "[stitch-fin] same voxel\n"); }
    else {
      bool cached = false;
      auto fit = trans_field_.find(t_enter);
      if (fit != trans_field_.end()) {
        bool has_goal = fit->second.dist.count(goal_surf) > 0;
        fprintf(stderr, "[stitch-fin] trans_field_[%d] exists, |dist|=%zu, has_goal=%d\n",
                t_enter, fit->second.dist.size(), has_goal);
        if (has_goal) {
          auto seg = walkToTarget(fit->second, goal_surf, t_enter);
          if (!seg.empty()) {
            std::reverse(seg.begin(), seg.end());
            append(path, seg);
            cached = true;
            fprintf(stderr, "[stitch-fin] CACHED %zu voxels\n", seg.size());
          } else {
            fprintf(stderr, "[stitch-fin] walkToTarget failed\n");
          }
        }
      } else {
        fprintf(stderr, "[stitch-fin] NO trans_field_ for t_enter=%d\n", t_enter);
      }
      if (!cached) {
        int exp = 0;
        auto seg = voxelAstar(t_enter, goal_surf, -1, &exp, fallback_corridor);
        fprintf(stderr, "[stitch-fin] FALLBACK A* → %zu voxels, exp=%d\n",
                seg.size(), exp);
        if (seg.empty()) { fprintf(stderr, "[stitch-fin] FAIL\n"); return {}; }
        append(path, seg);
        total_fallback_exp += exp;
      }
    }
  }

  if (out_expanded) *out_expanded = total_fallback_exp;

  fprintf(stderr, "[stitch] %zu voxels | fallback_exp=%d\n",
          path.size(), total_fallback_exp);
  return path;
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
  const RoomRoute route = roomAstar(
    start_room, goal_room, start_surf, goal_surf,
    start_world, goal_world, floor_seq, robot_diameter_m);
  const std::vector<int> & room_seq = route.rooms;
  auto room_t1 = std::chrono::steady_clock::now();
  result.room_planning_ms =
    std::chrono::duration<double, std::milli>(room_t1 - room_t0).count();

  if (room_seq.empty()) {
    // Debug: print adjacency sizes for start/goal rooms
    fprintf(stderr,
      "[HierPlanner] Room routing failed: start_room=%d (adj=%zu, floor=%d), "
      "goal_room=%d (adj=%zu, floor=%d), n_rooms=%d, n_floors=%d, robot_d=%.3f\n",
      start_room,
      start_room >= 0 ? rg_.adjacency[start_room].size() : (size_t)0,
      start_room >= 0 ? rg_.rooms[start_room].floor_id : -1,
      goal_room,
      goal_room >= 0 ? rg_.adjacency[goal_room].size() : (size_t)0,
      goal_room >= 0 ? rg_.rooms[goal_room].floor_id : -1,
      (int)rg_.rooms.size(), (int)fg_.floors.size(), robot_diameter_m);
    // Room graph is disconnected — fall back to unrestricted voxel A*.
    fprintf(stderr, "[HierPlanner] Falling back to direct voxel A*\n");
    auto fallback_t0 = std::chrono::steady_clock::now();
    int fb_expanded = 0;
    auto fb_path = voxelAstar(start_surf, goal_surf, -1, &fb_expanded);
    auto fallback_t1 = std::chrono::steady_clock::now();
    if (fb_path.empty()) {
      result.message = "Room-level routing failed and voxel fallback also failed.";
      return result;
    }
    result.voxel_planning_ms =
      std::chrono::duration<double, std::milli>(fallback_t1 - fallback_t0).count();
    result.voxel_nodes_expanded = fb_expanded;
    const auto & graph_fb = vmap_->getSurfaceGraph();
    result.waypoints.reserve(fb_path.size());
    for (int idx : fb_path) {
      const auto & nd = graph_fb[idx];
      result.waypoints.push_back(vmap_->gridToWorld(nd.x, nd.y, nd.z));
    }
    result.path_length_m = computePathLength(fb_path);
    auto total_t1_fb = std::chrono::steady_clock::now();
    result.total_planning_ms =
      std::chrono::duration<double, std::milli>(total_t1_fb - total_t0).count();
    result.success = true;
    result.message = "OK (voxel fallback)";
    return result;
  }
  result.room_sequence = room_seq;

  // ── Step 3: Stitched-geodesic reconstruction ─────────────────────────────
  //
  // First-principles improvement: precomputeTransfers() already built the
  // complete shortest-path forest within every room.  Reuse it!  Online cost
  // for multi-room queries collapses from a corridor A* (~17k expansions) to
  // a handful of parent-chain walks plus table lookups.  See stitchPath()
  // for details.  Falls back to corridor A* whenever stitching is unavailable
  // (single-room query, or a room with > kMaxEdgesForDijkstra incident edges
  // whose trans_field_ was skipped during precompute).
  auto voxel_t0 = std::chrono::steady_clock::now();
  int total_expanded = 0;
  std::vector<int> all_surf_path;
  bool used_stitch = false;

  if (room_seq.size() >= 2) {
    // Build corridor set for fallback A* segments (same as corridor A*)
    std::unordered_set<int> stitch_corridor(room_seq.begin(), room_seq.end());
    all_surf_path = stitchPath(start_surf, goal_surf, route, &total_expanded, &stitch_corridor);
    if (!all_surf_path.empty()) used_stitch = true;
  }

  if (all_surf_path.empty()) {
    // Fallback: original corridor A* (also handles single-room case).
    if (room_seq.size() <= 1) {
      all_surf_path = voxelAstar(start_surf, goal_surf, -1, &total_expanded);
    } else {
      std::unordered_set<int> corridor(room_seq.begin(), room_seq.end());
      all_surf_path = voxelAstar(start_surf, goal_surf, -1, &total_expanded, &corridor);
      if (all_surf_path.empty()) {
        all_surf_path = voxelAstar(start_surf, goal_surf, -1, &total_expanded);
      }
    }
  }

  auto voxel_t1 = std::chrono::steady_clock::now();
  result.voxel_planning_ms =
    std::chrono::duration<double, std::milli>(voxel_t1 - voxel_t0).count();
  result.voxel_nodes_expanded = total_expanded;  // 0 when stitch path was used

  if (all_surf_path.empty()) {
    result.message = "Step 3 failed (stitch + corridor A* both empty).";
    return result;
  }
  if (used_stitch) {
    fprintf(stderr, "[stitch] reused %zu voxels from precomputed segments\n",
            all_surf_path.size());
  }

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
