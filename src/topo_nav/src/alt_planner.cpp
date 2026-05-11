#include "topo_nav/alt_planner.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <vector>

namespace topo_nav
{

ALTPlanner::ALTPlanner(
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap,
  double w_up, double w_down, double w_obs)
: vmap_(std::move(vmap)), w_up_(w_up), w_down_(w_down), w_obs_(w_obs)
{}

int ALTPlanner::snapToSurface(
  const std::tuple<double, double, double> & world, bool snap) const
{
  auto [wx, wy, wz] = world;
  auto [ix, iy, iz] = vmap_->worldToGrid(wx, wy, wz);
  if (!vmap_->isValidIdx(ix, iy, iz)) return -1;
  int idx = vmap_->coordToSurfaceIdx(ix, iy, iz);
  if (idx >= 0) return idx;
  if (!snap) return -1;
  for (int r : {5, 20, 50}) {
    auto [nx, ny, nz] = vmap_->findNearestValidSurface(ix, iy, iz, r);
    if (nx >= 0) return vmap_->coordToSurfaceIdx(nx, ny, nz);
  }
  return -1;
}

double ALTPlanner::edgeCost(int s1, int s2) const
{
  const auto & graph = vmap_->getSurfaceGraph();
  const double res = vmap_->getResolution();
  const auto & nf = graph[s1];
  const auto & nt = graph[s2];
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
}

double ALTPlanner::heuristic(int v, int goal) const
{
  double h = 0.0;
  for (int l = 0; l < num_landmarks_; ++l) {
    const double d_lv = landmark_dist_[l * num_surface_ + v];
    const double d_lg = landmark_dist_[l * num_surface_ + goal];
    h = std::max(h, std::abs(d_lg - d_lv));
  }
  return h;
}

double ALTPlanner::computePathLength(const std::vector<int> & path) const
{
  const auto & graph = vmap_->getSurfaceGraph();
  const double res = vmap_->getResolution();
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = graph[path[i-1]];
    const auto & b = graph[path[i]];
    const double dx = (b.x - a.x) * res;
    const double dy = (b.y - a.y) * res;
    const double dz = (b.z - a.z) * res;
    len += std::sqrt(dx*dx + dy*dy + dz*dz);
  }
  return len;
}

void ALTPlanner::precompute(int num_landmarks)
{
  auto t0 = std::chrono::steady_clock::now();

  const auto & graph    = vmap_->getSurfaceGraph();
  const auto & all_nbrs = vmap_->getAllNeighbors();
  const int n = static_cast<int>(graph.size());
  num_surface_ = n;
  num_landmarks_ = num_landmarks;

  // Farthest-point sampling for landmark selection
  landmark_surf_.resize(num_landmarks);
  std::mt19937 rng(42);
  landmark_surf_[0] = rng() % n;

  // Temp: min distance from any selected landmark
  std::vector<double> min_dist(n, std::numeric_limits<double>::infinity());

  // Allocate landmark distance table
  landmark_dist_.assign(
    static_cast<size_t>(num_landmarks) * n,
    std::numeric_limits<double>::infinity());

  for (int l = 0; l < num_landmarks; ++l) {
    const int src = landmark_surf_[l];

    // Dijkstra from this landmark
    std::vector<double> & dist_l = landmark_dist_;  // alias for indexing
    const size_t offset = static_cast<size_t>(l) * n;

    using QE = std::pair<double, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    dist_l[offset + src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
      auto [d, u] = pq.top(); pq.pop();
      if (d > dist_l[offset + u]) continue;

      const auto & nd = graph[u];
      for (int ni = 0; ni < nd.neighbor_count; ++ni) {
        const int nbr = all_nbrs[nd.neighbor_start + ni];
        const double nd_cost = d + edgeCost(u, nbr);
        if (nd_cost < dist_l[offset + nbr]) {
          dist_l[offset + nbr] = nd_cost;
          pq.push({nd_cost, nbr});
        }
      }
    }

    // Update min_dist for farthest-point selection of next landmark
    if (l + 1 < num_landmarks) {
      int best_next = -1;
      double best_val = -1.0;
      for (int v = 0; v < n; ++v) {
        min_dist[v] = std::min(min_dist[v], dist_l[offset + v]);
        if (min_dist[v] > best_val && min_dist[v] < 1e18) {
          best_val = min_dist[v];
          best_next = v;
        }
      }
      if (best_next >= 0) {
        landmark_surf_[l + 1] = best_next;
      } else {
        // Fallback: random
        landmark_surf_[l + 1] = rng() % n;
      }
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  precompute_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

HierarchicalPath ALTPlanner::plan(
  const std::tuple<double, double, double> & start_world,
  const std::tuple<double, double, double> & goal_world,
  float /*robot_diameter_m*/,
  bool snap_to_surface)
{
  HierarchicalPath result;
  auto total_t0 = std::chrono::steady_clock::now();

  // Snap
  auto snap_t0 = std::chrono::steady_clock::now();
  const int start_surf = snapToSurface(start_world, snap_to_surface);
  const int goal_surf  = snapToSurface(goal_world, snap_to_surface);
  auto snap_t1 = std::chrono::steady_clock::now();
  result.snap_ms = std::chrono::duration<double, std::milli>(snap_t1 - snap_t0).count();

  if (start_surf < 0 || goal_surf < 0) {
    result.message = "Cannot snap start/goal to surface.";
    return result;
  }

  const auto & graph    = vmap_->getSurfaceGraph();
  const auto & all_nbrs = vmap_->getAllNeighbors();
  const int n = num_surface_;
  const double INF = std::numeric_limits<double>::infinity();

  // A* with ALT heuristic
  auto voxel_t0 = std::chrono::steady_clock::now();

  std::vector<double> g(n, INF);
  std::vector<int> came_from(n, -1);
  std::vector<bool> closed(n, false);

  using QE = std::tuple<double, double, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;

  g[start_surf] = 0.0;
  open.emplace(heuristic(start_surf, goal_surf), 0.0, start_surf);
  int expanded = 0;

  while (!open.empty()) {
    auto [_, cur_g, cur] = open.top(); open.pop();
    if (closed[cur]) continue;

    if (cur == goal_surf) {
      // Reconstruct
      std::vector<int> path;
      int nd = goal_surf;
      while (nd >= 0) {
        path.push_back(nd);
        if (nd == start_surf) break;
        nd = came_from[nd];
      }
      std::reverse(path.begin(), path.end());

      auto voxel_t1 = std::chrono::steady_clock::now();
      result.voxel_planning_ms =
        std::chrono::duration<double, std::milli>(voxel_t1 - voxel_t0).count();
      result.voxel_nodes_expanded = expanded;

      result.waypoints.reserve(path.size());
      for (int idx : path) {
        const auto & ndd = graph[idx];
        result.waypoints.push_back(vmap_->gridToWorld(ndd.x, ndd.y, ndd.z));
      }
      result.path_length_m = computePathLength(path);

      auto total_t1 = std::chrono::steady_clock::now();
      result.total_planning_ms =
        std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
      result.success = true;
      result.message = "OK (ALT A*)";
      return result;
    }

    closed[cur] = true;
    ++expanded;

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];
      if (closed[nbr]) continue;

      const double tentative_g = cur_g + edgeCost(cur, nbr);
      if (tentative_g < g[nbr]) {
        g[nbr] = tentative_g;
        came_from[nbr] = cur;
        open.emplace(tentative_g + heuristic(nbr, goal_surf), tentative_g, nbr);
      }
    }
  }

  auto voxel_t1 = std::chrono::steady_clock::now();
  result.voxel_planning_ms =
    std::chrono::duration<double, std::milli>(voxel_t1 - voxel_t0).count();
  result.voxel_nodes_expanded = expanded;
  result.message = "ALT A* found no path.";
  return result;
}

}  // namespace topo_nav
