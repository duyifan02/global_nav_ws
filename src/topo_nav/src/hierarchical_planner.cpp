#include "topo_nav/hierarchical_planner.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
  transfer_table_.clear();
  min_exit_cost_.clear();
  precomputeTransfers();
}

// ─────────────────────────────────────────────────────────────────────────────
// HPA* precomputation: intra-room transition-to-transition voxel paths
// ─────────────────────────────────────────────────────────────────────────────
void HierarchicalPlanner::precomputeTransfers()
{
  const double INF = std::numeric_limits<double>::infinity();

  for (int r = 0; r < rg_.num_rooms; ++r) {
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
    if (incident.size() < 2) continue;  // dead-end room: nothing to transfer

    auto & tmap = transfer_table_[r];
    auto & mmap = min_exit_cost_[r];

    for (size_t ii = 0; ii < incident.size(); ++ii) {
      for (size_t jj = 0; jj < incident.size(); ++jj) {
        if (ii == jj) continue;
        const auto [ei, t_i] = incident[ii];
        const auto [ej, t_j] = incident[jj];

        TransferEntry entry;
        auto path = voxelAstar(t_i, t_j, r);
        if (!path.empty()) {
          entry.cost      = computePathLength(path);
          entry.surf_path = std::move(path);
        } else {
          entry.cost = INF;
        }

        // Track minimum exit cost for edge ej across all possible entries
        auto it = mmap.find(ej);
        if (it == mmap.end() || entry.cost < it->second) {
          mmap[ej] = entry.cost;
        }

        tmap[ei][ej] = std::move(entry);
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

  // Heuristic: absolute floor index difference (admissible)
  auto h = [&](int f) {
    return std::abs(f - goal_floor) * 3.0;  // rough cost per floor
  };

  while (!open.empty()) {
    auto [_, cur] = open.top(); open.pop();
    if (closed[cur]) continue;
    if (cur == goal_floor) {
      // Reconstruct
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
  return {};  // unreachable
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
  const double INF = std::numeric_limits<double>::infinity();

  // Build allowed room set: rooms on allowed floors + staircase rooms that
  // link those floors.
  std::unordered_set<int> floor_set(allowed_floors.begin(), allowed_floors.end());
  std::vector<bool> allowed(R, false);
  for (int r = 0; r < R; ++r) {
    if (floor_set.count(rg_.rooms[r].floor_id)) {
      allowed[r] = true;
    }
    // Staircase rooms connecting any two allowed floors are always allowed.
    if (rg_.rooms[r].type == RoomType::STAIRCASE) {
      allowed[r] = true;
    }
  }
  // Ensure start and goal are accessible.
  allowed[start_room] = true;
  allowed[goal_room]  = true;

  std::vector<double> g(R, INF);
  std::vector<int>    came_from(R, -1);
  std::vector<bool>   closed(R, false);

  using QElem = std::pair<double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  g[start_room] = 0.0;
  open.emplace(0.0, start_room);

  // Heuristic: Euclidean distance between room centroids
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
      if (edge.passage_width_m < robot_diameter_m) continue;  // too narrow

      const auto & rm_cur = rg_.rooms[cur];
      const auto & rm_nbr = rg_.rooms[nbr];

      // Within-room traversal cost: use precomputed minimum if available,
      // otherwise fall back to Euclidean centroid→transition estimate.
      double within_cost;
      {
        const auto r_it = min_exit_cost_.find(cur);
        if (r_it != min_exit_cost_.end()) {
          const auto c_it = r_it->second.find(eidx);
          if (c_it != r_it->second.end()) {
            if (c_it->second >= std::numeric_limits<double>::infinity()) {
              continue;  // No valid path through this exit
            }
            within_cost = c_it->second;
          } else {
            // Edge not in table (single-incident room): Euclidean fallback
            const double dtx = edge.trans_wx - rm_cur.cx;
            const double dty = edge.trans_wy - rm_cur.cy;
            const double dtz = edge.trans_wz - rm_cur.cz;
            within_cost = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
          }
        } else {
          // Room has no precomputed transfers (single edge): Euclidean fallback
          const double dtx = edge.trans_wx - rm_cur.cx;
          const double dty = edge.trans_wy - rm_cur.cy;
          const double dtz = edge.trans_wz - rm_cur.cz;
          within_cost = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
        }
      }

      // Cost from transition point to neighbor centroid (Euclidean approximation)
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
// ─────────────────────────────────────────────────────────────────────────────
std::vector<int> HierarchicalPlanner::voxelAstar(
  int start_surf, int goal_surf, int allowed_room_id) const
{
  if (start_surf == goal_surf) return {start_surf};

  const auto & graph    = vmap_->getSurfaceGraph();
  const auto & all_nbrs = vmap_->getAllNeighbors();
  const int n = static_cast<int>(graph.size());
  const double res = vmap_->getResolution();
  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> g(n, INF);
  std::vector<int>    came_from(n, -1);
  std::vector<char>   closed(n, 0);

  using QElem = std::tuple<double, double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  // Heuristic: Euclidean distance (admissible with ε=1)
  const auto & gn = graph[goal_surf];
  auto h = [&](int idx) -> double {
    const auto & nd = graph[idx];
    const double dx = (nd.x - gn.x) * res;
    const double dy = (nd.y - gn.y) * res;
    const double dz = (nd.z - gn.z) * res;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  // Edge cost: same as JPS3D (Euclidean + height penalty + clearance penalty)
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

  g[start_surf] = 0.0;
  open.emplace(h(start_surf), 0.0, start_surf);

  while (!open.empty()) {
    auto [_, cur_g, cur] = open.top(); open.pop();
    if (closed[cur]) continue;
    if (cur == goal_surf) {
      std::vector<int> path;
      int nd = goal_surf;
      while (nd >= 0) {
        path.push_back(nd);
        if (nd == start_surf) break;
        nd = came_from[nd];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    closed[cur] = 1;

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];
      if (closed[nbr]) continue;

      // Room filter: only expand nodes in the allowed room (or start/goal)
      if (allowed_room_id >= 0 &&
          nbr != goal_surf &&
          rg_.room_labels[nbr] != allowed_room_id)
      {
        continue;
      }

      const double tentative_g = g[cur] + edge_cost(cur, nbr);
      if (tentative_g < g[nbr]) {
        g[nbr]        = tentative_g;
        came_from[nbr] = cur;
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

  // ── Step 1: Floor-level routing ───────────────────────────────────────────
  auto floor_t0 = std::chrono::steady_clock::now();
  const std::vector<int> floor_seq = floorAstar(start_floor, goal_floor);
  auto floor_t1 = std::chrono::steady_clock::now();
  result.floor_planning_ms =
    std::chrono::duration<double, std::milli>(floor_t1 - floor_t0).count();

  if (floor_seq.empty()) {
    result.message = "Floor-level routing failed (disconnected floors?).";
    return result;
  }
  result.floor_sequence = floor_seq;

  // ── Step 2: Room-level routing ────────────────────────────────────────────
  auto room_t0 = std::chrono::steady_clock::now();
  const std::vector<int> room_seq =
    roomAstar(start_room, goal_room, floor_seq, robot_diameter_m);
  auto room_t1 = std::chrono::steady_clock::now();
  result.room_planning_ms =
    std::chrono::duration<double, std::milli>(room_t1 - room_t0).count();

  if (room_seq.empty()) {
    result.message = "Room-level routing failed (no passable path?).";
    return result;
  }
  result.room_sequence = room_seq;

  // ── Step 3: Voxel-level refinement ────────────────────────────────────────
  // Build the ordered list of waypoints (surface graph indices):
  //   start_surf, t_exit(r0→r1), t_entry(r1), t_exit(r1→r2), t_entry(r2), ..., goal_surf
  // Then for each consecutive pair within a room, plan restricted A*.
  auto voxel_t0 = std::chrono::steady_clock::now();

  const int P = static_cast<int>(room_seq.size());
  std::vector<int> all_surf_path;
  all_surf_path.push_back(start_surf);

  int prev_entry = start_surf;  // entry point into the current room
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

    // Middle rooms: try precomputed HPA* transfer path
    if (k > 0 && k < P - 1) {
      const int prev_room    = room_seq[k - 1];
      int entry_eidx = -1;
      const auto it = rg_.adjacency.find(cur_room);
      if (it != rg_.adjacency.end()) {
        for (auto & [nbr, ei] : it->second) {
          if (nbr == prev_room) { entry_eidx = ei; break; }
        }
      }

      if (entry_eidx >= 0) {
        const auto r_it = transfer_table_.find(cur_room);
        if (r_it != transfer_table_.end()) {
          const auto in_it = r_it->second.find(entry_eidx);
          if (in_it != r_it->second.end()) {
            const auto out_it = in_it->second.find(exit_eidx);
            if (out_it != in_it->second.end() &&
                !out_it->second.surf_path.empty())
            {
              seg = out_it->second.surf_path;  // precomputed path (copy)
            }
          }
        }
      }
    }

    // First / last room (or middle room with no precomputed path): runtime A*
    if (seg.empty()) {
      seg = voxelAstar(prev_entry, exit_surf, cur_room);
      if (seg.empty()) {
        seg = voxelAstar(prev_entry, exit_surf, -1);  // unrestricted fallback
      }
    }

    if (seg.empty()) {
      result.message = "Voxel-level A* failed in room " + std::to_string(cur_room);
      return result;
    }

    // ── Append segment to path ────────────────────────────────────────────
    // Skip the first element if it duplicates the last appended voxel.
    const size_t start_si = (seg.front() == all_surf_path.back()) ? 1u : 0u;
    for (size_t si = start_si; si < seg.size(); ++si) {
      all_surf_path.push_back(seg[si]);
    }
    total_expanded += static_cast<int>(seg.size());

    // ── Set up entry for next room ────────────────────────────────────────
    if (k < P - 1) {
      const int next_room = room_seq[k + 1];
      const auto & graph    = vmap_->getSurfaceGraph();
      const auto & all_nbrs = vmap_->getAllNeighbors();
      const auto & nd = graph[exit_surf];

      int entry_next = -1;
      // Prefer neighbor with highest D (clearance) in next room
      int best_d = -1;
      for (int ni = 0; ni < nd.neighbor_count; ++ni) {
        const int q = all_nbrs[nd.neighbor_start + ni];
        if (rg_.room_labels[q] == next_room) {
          if (graph[q].distance > best_d) {
            best_d    = graph[q].distance;
            entry_next = q;
          }
        }
      }
      if (entry_next < 0) {
        // Fallback: canonical transition surface on next room's side
        if (exit_eidx >= 0) {
          const auto & edge = rg_.edges[exit_eidx];
          entry_next = (edge.room_i == next_room) ? edge.transition_surf_i
                                                   : edge.transition_surf_j;
        }
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
