#include "topo_nav/prm_planner.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <vector>

namespace topo_nav
{

PRMPlanner::PRMPlanner(
  std::shared_ptr<jps_planner_3d::VoxelMap> vmap,
  double w_up, double w_down, double w_obs)
: vmap_(std::move(vmap)), w_up_(w_up), w_down_(w_down), w_obs_(w_obs)
{}

int PRMPlanner::snapToSurface(
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

double PRMPlanner::edgeCost(int s1, int s2) const
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

// Unrestricted A* on surface graph (replicates HierarchicalPlanner::voxelAstar
// with allowed_room_id=-1).
std::vector<int> PRMPlanner::localAstar(int start_surf, int goal_surf,
                                        int* out_expanded) const
{
  if (out_expanded) *out_expanded = 0;
  if (start_surf == goal_surf) return {start_surf};

  const auto & graph    = vmap_->getSurfaceGraph();
  const auto & all_nbrs = vmap_->getAllNeighbors();
  const double res = vmap_->getResolution();
  const double INF = std::numeric_limits<double>::infinity();

  ++astar_gen_;
  if (astar_gen_ == 0u) {
    std::fill(astar_g_gen_.begin(), astar_g_gen_.end(), 0u);
    std::fill(astar_cl_gen_.begin(), astar_cl_gen_.end(), 0u);
    ++astar_gen_;
  }
  const uint32_t gen = astar_gen_;

  const auto & gn = graph[goal_surf];
  auto h = [&](int idx) -> double {
    const auto & nd = graph[idx];
    const double dx = (nd.x - gn.x) * res;
    const double dy = (nd.y - gn.y) * res;
    const double dz = (nd.z - gn.z) * res;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  };

  using QElem = std::tuple<double, double, int>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

  astar_g_gen_[start_surf] = gen;
  astar_g_[start_surf] = 0.0;
  astar_came_from_[start_surf] = -1;
  open.emplace(h(start_surf), 0.0, start_surf);

  while (!open.empty()) {
    auto [_, cur_g, cur] = open.top(); open.pop();
    if (astar_cl_gen_[cur] == gen) continue;
    if (cur == goal_surf) {
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
    astar_cl_gen_[cur] = gen;
    if (out_expanded) ++(*out_expanded);

    const auto & nd = graph[cur];
    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int nbr = all_nbrs[nd.neighbor_start + ni];
      if (astar_cl_gen_[nbr] == gen) continue;
      const double tentative_g = cur_g + edgeCost(cur, nbr);
      const double nbr_g = (astar_g_gen_[nbr] == gen) ? astar_g_[nbr] : INF;
      if (tentative_g < nbr_g) {
        astar_g_gen_[nbr] = gen;
        astar_g_[nbr] = tentative_g;
        astar_came_from_[nbr] = cur;
        open.emplace(tentative_g + h(nbr), tentative_g, nbr);
      }
    }
  }
  return {};
}

void PRMPlanner::buildRoadmap(int num_samples, int k_neighbors, double /*max_edge_len*/)
{
  auto t0 = std::chrono::steady_clock::now();

  const auto & graph = vmap_->getSurfaceGraph();
  const int n = static_cast<int>(graph.size());
  const double res = vmap_->getResolution();

  // Init dirty-array state
  astar_g_.resize(n);
  astar_came_from_.resize(n);
  astar_g_gen_.assign(n, 0u);
  astar_cl_gen_.assign(n, 0u);
  astar_gen_ = 0u;

  // Sample surface indices uniformly
  num_samples = std::min(num_samples, n);
  std::vector<int> indices(n);
  std::iota(indices.begin(), indices.end(), 0);
  std::mt19937 rng(42);
  std::shuffle(indices.begin(), indices.end(), rng);
  node_surf_.assign(indices.begin(), indices.begin() + num_samples);
  num_nodes_ = num_samples;

  // Build k-nearest-neighbor edges using Euclidean distance
  adj_.resize(num_nodes_);

  // Precompute world positions for roadmap nodes
  std::vector<std::tuple<double,double,double>> node_pos(num_nodes_);
  for (int i = 0; i < num_nodes_; ++i) {
    const auto & nd = graph[node_surf_[i]];
    node_pos[i] = vmap_->gridToWorld(nd.x, nd.y, nd.z);
  }

  for (int i = 0; i < num_nodes_; ++i) {
    auto [xi, yi, zi] = node_pos[i];

    // Find k nearest
    std::vector<std::pair<double, int>> dists;
    dists.reserve(num_nodes_);
    for (int j = 0; j < num_nodes_; ++j) {
      if (j == i) continue;
      auto [xj, yj, zj] = node_pos[j];
      double d = std::sqrt((xj-xi)*(xj-xi) + (yj-yi)*(yj-yi) + (zj-zi)*(zj-zi));
      dists.emplace_back(d, j);
    }
    std::partial_sort(dists.begin(),
      dists.begin() + std::min(k_neighbors, static_cast<int>(dists.size())),
      dists.end());

    const int k = std::min(k_neighbors, static_cast<int>(dists.size()));
    for (int ki = 0; ki < k; ++ki) {
      int j = dists[ki].second;
      // Compute true edge cost on the surface graph (Euclidean approximation)
      double cost = dists[ki].first;
      adj_[i].push_back({j, cost});
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  build_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

HierarchicalPath PRMPlanner::plan(
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

  const auto & graph = vmap_->getSurfaceGraph();
  const double res = vmap_->getResolution();

  // Find nearest roadmap nodes to start and goal
  auto findNearest = [&](int surf_idx) -> int {
    const auto & nd = graph[surf_idx];
    auto [wx, wy, wz] = vmap_->gridToWorld(nd.x, nd.y, nd.z);
    double best_d = std::numeric_limits<double>::infinity();
    int best = -1;
    for (int i = 0; i < num_nodes_; ++i) {
      const auto & rn = graph[node_surf_[i]];
      auto [rx, ry, rz] = vmap_->gridToWorld(rn.x, rn.y, rn.z);
      double d = std::sqrt((rx-wx)*(rx-wx) + (ry-wy)*(ry-wy) + (rz-wz)*(rz-wz));
      if (d < best_d) { best_d = d; best = i; }
    }
    return best;
  };

  const int start_rm = findNearest(start_surf);
  const int goal_rm  = findNearest(goal_surf);
  if (start_rm < 0 || goal_rm < 0) {
    result.message = "Cannot find nearest PRM node.";
    return result;
  }

  // Dijkstra on roadmap
  auto voxel_t0 = std::chrono::steady_clock::now();
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> dist(num_nodes_, INF);
  std::vector<int> prev(num_nodes_, -1);

  using QE = std::pair<double, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  dist[start_rm] = 0.0;
  pq.push({0.0, start_rm});

  while (!pq.empty()) {
    auto [d, u] = pq.top(); pq.pop();
    if (d > dist[u]) continue;
    if (u == goal_rm) break;
    for (const auto & e : adj_[u]) {
      double nd = d + e.cost;
      if (nd < dist[e.to]) {
        dist[e.to] = nd;
        prev[e.to] = u;
        pq.push({nd, e.to});
      }
    }
  }

  if (dist[goal_rm] >= INF) {
    result.message = "PRM roadmap has no path.";
    return result;
  }

  // Reconstruct roadmap path
  std::vector<int> rm_path;
  for (int u = goal_rm; u >= 0; u = prev[u]) {
    rm_path.push_back(u);
  }
  std::reverse(rm_path.begin(), rm_path.end());

  // Build full voxel path: start → roadmap[0], roadmap[i] → roadmap[i+1], roadmap[last] → goal
  int total_expanded = 0;
  std::vector<int> full_path;

  // Start to first roadmap node
  {
    int exp = 0;
    auto seg = localAstar(start_surf, node_surf_[rm_path[0]], &exp);
    total_expanded += exp;
    if (seg.empty()) {
      result.message = "PRM: local A* start→roadmap failed.";
      return result;
    }
    full_path.insert(full_path.end(), seg.begin(), seg.end());
  }

  // Roadmap segments
  for (size_t i = 0; i + 1 < rm_path.size(); ++i) {
    int exp = 0;
    auto seg = localAstar(node_surf_[rm_path[i]], node_surf_[rm_path[i+1]], &exp);
    total_expanded += exp;
    if (seg.empty()) {
      result.message = "PRM: local A* roadmap segment failed.";
      return result;
    }
    // Skip first node (already at end of full_path)
    full_path.insert(full_path.end(), seg.begin() + 1, seg.end());
  }

  // Last roadmap node to goal
  {
    int exp = 0;
    auto seg = localAstar(node_surf_[rm_path.back()], goal_surf, &exp);
    total_expanded += exp;
    if (seg.empty()) {
      result.message = "PRM: local A* roadmap→goal failed.";
      return result;
    }
    full_path.insert(full_path.end(), seg.begin() + 1, seg.end());
  }

  auto voxel_t1 = std::chrono::steady_clock::now();
  result.voxel_planning_ms =
    std::chrono::duration<double, std::milli>(voxel_t1 - voxel_t0).count();
  result.voxel_nodes_expanded = total_expanded;

  // Convert to world coords
  result.waypoints.reserve(full_path.size());
  for (int idx : full_path) {
    const auto & nd = graph[idx];
    result.waypoints.push_back(vmap_->gridToWorld(nd.x, nd.y, nd.z));
  }

  // Path length
  double len = 0.0;
  for (size_t i = 1; i < full_path.size(); ++i) {
    const auto & a = graph[full_path[i-1]];
    const auto & b = graph[full_path[i]];
    const double dx = (b.x - a.x) * res;
    const double dy = (b.y - a.y) * res;
    const double dz = (b.z - a.z) * res;
    len += std::sqrt(dx*dx + dy*dy + dz*dz);
  }
  result.path_length_m = len;

  auto total_t1 = std::chrono::steady_clock::now();
  result.total_planning_ms =
    std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
  result.success = true;
  result.message = "OK (PRM)";
  return result;
}

}  // namespace topo_nav
