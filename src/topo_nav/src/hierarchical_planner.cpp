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

  trans_field_.clear();

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

    // Guard against combinatorial explosion in highly-connected rooms.
    // With Scheme 2 (Dijkstra), cost is K Dijkstra calls instead of K*(K-1) A*,
    // but very large rooms with many edges still dominate precompute time.
    // Cap at ~15 edges (each Dijkstra visits all room voxels).
    static constexpr size_t kMaxEdgesForDijkstra = 15;
    if (incident.size() > kMaxEdgesForDijkstra) {
      const auto & graph_skip = vmap_->getSurfaceGraph();
      int skip_vx = 0;
      for (int vi = 0; vi < static_cast<int>(graph_skip.size()); ++vi)
        if (rg_.room_labels[vi] == r) ++skip_vx;
      fprintf(stderr,
        "[HPA*] room %3d: %4d voxels, %zu edges — skipping precompute "
        "(> %zu edge cap); roomAstar will use Euclidean fallback\n",
        r, skip_vx, incident.size(), kMaxEdgesForDijkstra);
      continue;
    }

    const auto r_t0 = std::chrono::steady_clock::now();
    int room_voxels = 0;
    const auto & graph_diag = vmap_->getSurfaceGraph();
    for (int vi = 0; vi < static_cast<int>(graph_diag.size()); ++vi)
      if (rg_.room_labels[vi] == r) ++room_voxels;

    // Scheme 2: k single-source Dijkstra replaces k*(k-1) pairwise A*.
    // Each Dijkstra gives distances from one transition to ALL room voxels,
    // yielding transfer costs + reusable distance fields in one pass.
    for (size_t ii = 0; ii < incident.size(); ++ii) {
      const auto [ei, t_i] = incident[ii];

      // Run exhaustive Dijkstra from t_i within room r
      TransitionField field;
      roomDijkstra(t_i, r, field.dist, field.parent);

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
std::vector<int> HierarchicalPlanner::roomAstar(
  int start_room,
  int goal_room,
  const std::tuple<double, double, double> & start_world,
  const std::tuple<double, double, double> & goal_world,
  const std::vector<int> & allowed_floors,
  float /*robot_diameter_m*/) const
{
  if (start_room == goal_room) return {start_room};

  const int R = rg_.num_rooms;
  const int E = transfer_E_;
  if (E == 0 || R == 0) return {};

  const double INF = std::numeric_limits<double>::infinity();

  std::unordered_set<int> floor_set(allowed_floors.begin(), allowed_floors.end());
  auto room_allowed = [&](int r) -> bool {
    if (r == start_room || r == goal_room) return true;
    if (rg_.rooms[r].type == RoomType::STAIRCASE) return true;
    return (bool)floor_set.count(rg_.rooms[r].floor_id);
  };

  // State: entry_edge_idx * R + room_id  (came_from value -1 = virtual start)
  const int STATE_COUNT = E * R;
  std::vector<double> g(STATE_COUNT, INF);
  std::vector<int>    came_from(STATE_COUNT, -2);  // -2 = not reached
  std::vector<bool>   closed(STATE_COUNT, false);

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  // Heuristic: Euclidean from edge transition to the actual goal world position.
  // This is admissible: straight-line distance ≤ any realizable path length.
  // Using goal_world (not goal_room centroid) prevents inadmissibility when
  // goal_surf is closer to the current transition than the centroid is.
  const auto [gx, gy, gz] = goal_world;
  auto h = [&](int ei) -> double {
    const auto & edge = rg_.edges[ei];
    const double dx = edge.trans_wx - gx;
    const double dy = edge.trans_wy - gy;
    const double dz = edge.trans_wz - gz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  // ── Initialise: from start_room push all its exit edges ──────────────────
  // Cost = Euclidean from actual start_world to each transition.
  // Using start_world (not room centroid) gives a tighter cost estimate for
  // the start-room traversal.
  const auto [sx, sy, sz] = start_world;
  const auto adj_start = rg_.adjacency.find(start_room);
  if (adj_start == rg_.adjacency.end()) return {};

  for (auto & [nbr, eidx] : adj_start->second) {
    if (!room_allowed(nbr)) continue;
    const auto & edge = rg_.edges[eidx];
    const double dtx = edge.trans_wx - sx;
    const double dty = edge.trans_wy - sy;
    const double dtz = edge.trans_wz - sz;
    const double init_cost = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);

    const int state = eidx * R + nbr;
    if (init_cost < g[state]) {
      g[state]          = init_cost;
      came_from[state]  = -1;  // came from virtual start
      open.emplace(init_cost + h(eidx), state);
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
      // Reconstruct room sequence by following came_from chain
      std::vector<int> room_path;
      int s = cur_state;
      while (s >= 0) {          // -1 = virtual start sentinel → stop
        room_path.push_back(s % R);
        s = came_from[s];
      }
      room_path.push_back(start_room);
      std::reverse(room_path.begin(), room_path.end());

      // Debug: print room sequence with voxel counts
      fprintf(stderr, "[roomAstar] seq(%zu rooms): ", room_path.size());
      const auto & graph_ra = vmap_->getSurfaceGraph();
      for (int ri : room_path) {
        int cnt = 0;
        for (int vi = 0; vi < static_cast<int>(graph_ra.size()); ++vi)
          if (rg_.room_labels[vi] == ri) ++cnt;
        fprintf(stderr, "%d(%dvx) ", ri, cnt);
      }
      fprintf(stderr, "\n");

      return room_path;
    }

    // ── Expand: traverse cur_room from entry_ei to each exit edge ────────
    const auto adj_it = rg_.adjacency.find(cur_room);
    if (adj_it == rg_.adjacency.end()) continue;

    for (auto & [nbr, ejdx] : adj_it->second) {
      if (ejdx == entry_ei) continue;   // no immediate back-tracking
      if (!room_allowed(nbr)) continue;

      // Exact precomputed transfer cost: entry_ei → ejdx through cur_room.
      // Both edges are incident to cur_room, so this slot was filled during
      // precomputeTransfers().  Falls back to Euclidean if unavailable.
      double within_cost = transfer_cost_[entry_ei * E + ejdx];
      if (within_cost >= INF) {
        const auto & e_in  = rg_.edges[entry_ei];
        const auto & e_out = rg_.edges[ejdx];
        const double dx = e_out.trans_wx - e_in.trans_wx;
        const double dy = e_out.trans_wy - e_in.trans_wy;
        const double dz = e_out.trans_wz - e_in.trans_wz;
        within_cost = std::sqrt(dx * dx + dy * dy + dz * dz);
      }

      const double new_g     = g[cur_state] + within_cost;
      const int    next_state = ejdx * R + nbr;
      if (new_g < g[next_state]) {
        g[next_state]         = new_g;
        came_from[next_state] = cur_state;
        open.emplace(new_g + h(ejdx), next_state);
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
  int source_surf, int room_id,
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

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  astar_g_gen_[source_surf]     = gen;
  astar_g_[source_surf]         = 0.0;
  astar_came_from_[source_surf] = -1;
  open.emplace(0.0, source_surf);

  // Track visited nodes for efficient extraction (avoids scanning all n voxels)
  std::vector<int> visited;
  visited.reserve(4096);

  while (!open.empty()) {
    auto [cur_g, cur] = open.top(); open.pop();
    if (astar_cl_gen_[cur] == gen) continue;
    astar_cl_gen_[cur] = gen;
    visited.push_back(cur);

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];
      if (astar_cl_gen_[nbr] == gen) continue;

      // Room restriction (source itself is always accessible)
      if (room_id >= 0 && rg_.room_labels[nbr] != room_id) continue;

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

  // Extract results for visited nodes
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
    roomAstar(start_room, goal_room, start_world, goal_world, floor_seq, robot_diameter_m);
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

  // ── Step 3: Corridor A* (Scheme 1) ─────────────────────────────────────────
  //
  // Instead of running separate voxelAstar calls per room and stitching
  // segments together, we run a SINGLE A* from start to goal with the
  // room sequence as a corridor constraint.  Benefits:
  //   - One priority queue construction (eliminates per-room fixed overhead)
  //   - No path stitching artifacts at room boundaries
  //   - Natural cross-room shortcuts within the corridor
  //   - Code simplification: ~120 lines → ~15 lines
  auto voxel_t0 = std::chrono::steady_clock::now();
  int total_expanded = 0;
  std::vector<int> all_surf_path;

  if (room_seq.size() <= 1) {
    // Single room or empty: unrestricted search (allows cross-room shortcuts)
    all_surf_path = voxelAstar(start_surf, goal_surf, -1, &total_expanded);
  } else {
    // Multi-room: build corridor from room sequence, single A* within it
    std::unordered_set<int> corridor(room_seq.begin(), room_seq.end());
    all_surf_path = voxelAstar(start_surf, goal_surf, -1, &total_expanded, &corridor);
    // If corridor search fails (rare: room sequence too narrow), fall back to unrestricted
    if (all_surf_path.empty()) {
      all_surf_path = voxelAstar(start_surf, goal_surf, -1, &total_expanded);
    }
  }

  auto voxel_t1 = std::chrono::steady_clock::now();
  result.voxel_planning_ms =
    std::chrono::duration<double, std::milli>(voxel_t1 - voxel_t0).count();
  result.voxel_nodes_expanded = total_expanded;

  if (all_surf_path.empty()) {
    result.message = "Corridor A* failed (even with unrestricted fallback).";
    return result;
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
