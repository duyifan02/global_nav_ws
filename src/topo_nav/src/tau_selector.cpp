#include "topo_nav/tau_selector.hpp"
#include "topo_nav/room_segmenter.hpp"

#include <jps_planner_3d/voxel_map.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace topo_nav
{

// ─────────────────────────────────────────────────────────────────────────────
// Gini coefficient of a list of non-negative integers.
// Returns 0 for uniform, approaches 1 for maximally unequal.
// ─────────────────────────────────────────────────────────────────────────────
static double gini(const std::vector<int> & vals)
{
  if (vals.empty()) return 0.0;
  const int n = static_cast<int>(vals.size());
  std::vector<int> sorted(vals);
  std::sort(sorted.begin(), sorted.end());
  double sum = 0.0, cumsum = 0.0;
  for (int i = 0; i < n; ++i) {
    cumsum += sorted[i];
    sum += (2.0 * (i + 1) - n - 1) * sorted[i];
  }
  if (cumsum == 0.0) return 0.0;
  return sum / (n * cumsum);
}

// ─────────────────────────────────────────────────────────────────────────────
// Count connected components of the room adjacency graph.
// ─────────────────────────────────────────────────────────────────────────────
static int countAdjComponents(const RoomGraph & rg)
{
  if (rg.num_rooms == 0) return 0;
  std::vector<bool> visited(rg.num_rooms, false);
  int components = 0;
  for (int r = 0; r < rg.num_rooms; ++r) {
    if (visited[r]) continue;
    ++components;
    std::queue<int> q;
    q.push(r);
    visited[r] = true;
    while (!q.empty()) {
      int u = q.front(); q.pop();
      auto it = rg.adjacency.find(u);
      if (it == rg.adjacency.end()) continue;
      for (auto & [nbr, _] : it->second) {
        if (!visited[nbr]) {
          visited[nbr] = true;
          q.push(nbr);
        }
      }
    }
  }
  return components;
}

// ─────────────────────────────────────────────────────────────────────────────
// Evaluate one τ: segment rooms and collect lightweight stats.
// ─────────────────────────────────────────────────────────────────────────────
static TauStats evaluateTau(
  float tau,
  const PersistenceResult & pr,
  const jps_planner_3d::VoxelMap & vmap)
{
  TauStats s;
  s.tau = tau;

  RoomGraph rg = segmentRooms(pr, vmap, tau);

  s.num_rooms = rg.num_rooms;
  s.num_edges = static_cast<int>(rg.edges.size());
  s.num_portals = 2 * s.num_edges;

  const int V = static_cast<int>(vmap.getSurfaceGraph().size());

  // Per-room voxel counts
  std::vector<int> room_voxel_count(rg.num_rooms, 0);
  for (int vi = 0; vi < V; ++vi) {
    int r = rg.room_labels[vi];
    if (r >= 0 && r < rg.num_rooms) ++room_voxel_count[r];
  }

  // Per-room portal counts
  std::vector<int> room_portal_count(rg.num_rooms, 0);
  for (auto & [room_id, adj_list] : rg.adjacency) {
    if (room_id >= 0 && room_id < rg.num_rooms)
      room_portal_count[room_id] = static_cast<int>(adj_list.size());
  }

  // Aggregate stats
  for (int r = 0; r < rg.num_rooms; ++r) {
    if (room_voxel_count[r] > s.max_room_voxels)
      s.max_room_voxels = room_voxel_count[r];
    if (room_portal_count[r] > s.max_room_portals)
      s.max_room_portals = room_portal_count[r];
  }
  if (V > 0)
    s.max_room_fraction = static_cast<double>(s.max_room_voxels) / V;

  // Mean portals per room
  int total_portals = 0;
  for (int r = 0; r < rg.num_rooms; ++r) total_portals += room_portal_count[r];
  s.mean_portals_per_room = rg.num_rooms > 0 ? total_portals / rg.num_rooms : 0;

  // Gini coefficient of room sizes
  s.room_size_gini = gini(room_voxel_count);

  // Budget simulation (same formula as precomputeTransfers)
  const double res = vmap.getResolution();
  const double scale = std::max(1.0, (0.1 / res) * (0.1 / res));
  const size_t budget = static_cast<size_t>(5000000.0 * scale);

  int skip_rooms = 0;
  int skip_voxels = 0;
  double max_ratio = 0.0;
  int total_pairs = 0;
  int cached_pairs = 0;

  for (int r = 0; r < rg.num_rooms; ++r) {
    int k = room_portal_count[r];
    if (k < 2) continue;
    int pairs = k * (k - 1);
    total_pairs += pairs;

    size_t work = static_cast<size_t>(k) * room_voxel_count[r];
    double ratio = static_cast<double>(work) / budget;
    if (ratio > max_ratio) max_ratio = ratio;

    if (work > budget) {
      ++skip_rooms;
      skip_voxels += room_voxel_count[r];
    } else {
      cached_pairs += pairs;
    }
  }

  s.budget_skip_rooms = skip_rooms;
  s.skip_voxel_fraction = V > 0 ? static_cast<double>(skip_voxels) / V : 0.0;
  s.max_budget_ratio = max_ratio;
  s.coverage_est = total_pairs > 0
    ? static_cast<double>(cached_pairs) / total_pairs : 1.0;

  // Connected components
  s.tg_connected_est = countAdjComponents(rg);

  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rule-based scoring function.
//
// Design principles (from REVIEW_v2):
//   - Prefer high coverage (no budget skips)
//   - Prefer single connected component
//   - Penalize very large max room fraction (>30% of scene)
//   - Penalize too few rooms (poor abstraction, <5)
//   - Penalize very high room count (fragmented, TG too large)
//   - Use only scene stats, no ALT timing
// ─────────────────────────────────────────────────────────────────────────────
static double scoreTau(const TauStats & s)
{
  // Base score: coverage is king
  double score = 100.0 * s.coverage_est;

  // Penalty: disconnected components (each extra component = -10)
  if (s.tg_connected_est > 1)
    score -= 10.0 * (s.tg_connected_est - 1);

  // Penalty: budget-skipped rooms
  score -= 15.0 * s.budget_skip_rooms;

  // Penalty: max room fraction > 0.3 (large room dominates)
  if (s.max_room_fraction > 0.3)
    score -= 20.0 * (s.max_room_fraction - 0.3);

  // Penalty: too few rooms (< 5) → poor abstraction
  if (s.num_rooms < 5)
    score -= 10.0 * (5 - s.num_rooms);

  // Penalty: Gini > 0.7 (very unequal room sizes)
  if (s.room_size_gini > 0.7)
    score -= 10.0 * (s.room_size_gini - 0.7);

  // Slight preference for fewer rooms at equal coverage (simpler TG)
  score -= 0.01 * s.num_portals;

  return score;
}

// ─────────────────────────────────────────────────────────────────────────────
TauChoice selectTau(
  const PersistenceResult & pr,
  const jps_planner_3d::VoxelMap & vmap,
  const std::vector<float> & candidates)
{
  auto t0 = std::chrono::steady_clock::now();

  TauChoice result;
  result.candidates.reserve(candidates.size());

  for (float tau : candidates) {
    TauStats s = evaluateTau(tau, pr, vmap);
    s.score = scoreTau(s);
    result.candidates.push_back(s);

    fprintf(stderr,
      "[τ-select] τ=%.1f: %d rooms, %d edges, %d components, "
      "coverage=%.1f%%, max_room=%.1f%%, gini=%.2f, skip=%d, score=%.1f\n",
      tau, s.num_rooms, s.num_edges, s.tg_connected_est,
      s.coverage_est * 100.0, s.max_room_fraction * 100.0,
      s.room_size_gini, s.budget_skip_rooms, s.score);
  }

  // Pick highest score
  int best = 0;
  for (int i = 1; i < static_cast<int>(result.candidates.size()); ++i) {
    if (result.candidates[i].score > result.candidates[best].score)
      best = i;
  }

  result.selected_idx = best;
  result.selected_tau = result.candidates[best].tau;

  auto t1 = std::chrono::steady_clock::now();
  result.selection_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  fprintf(stderr,
    "[τ-select] Selected τ=%.1f (score=%.1f) in %.1f ms\n",
    result.selected_tau, result.candidates[best].score,
    result.selection_time_ms);

  return result;
}

}  // namespace topo_nav
