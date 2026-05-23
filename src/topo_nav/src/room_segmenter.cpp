#include "topo_nav/room_segmenter.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace topo_nav
{

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace
{

/// Compute the 2-D PCA aspect ratio sqrt(λ1 / λ2) of the room's (x,y) footprint.
/// Returns 1.0 for rooms with fewer than 3 voxels or zero minor axis.
float computePcaRatio(const std::vector<float> & xs, const std::vector<float> & ys)
{
  const int n = static_cast<int>(xs.size());
  if (n < 3) return 1.0f;

  // Centroid
  float mx = 0, my = 0;
  for (int i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; }
  mx /= n; my /= n;

  // Covariance
  float cxx = 0, cxy = 0, cyy = 0;
  for (int i = 0; i < n; ++i) {
    const float dx = xs[i] - mx;
    const float dy = ys[i] - my;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }
  cxx /= n; cxy /= n; cyy /= n;

  // Eigenvalues of 2×2 symmetric matrix
  const float trace = cxx + cyy;
  const float disc  = std::sqrt(std::max(0.f,
    (cxx - cyy) * (cxx - cyy) * 0.25f + cxy * cxy));
  const float lam1 = trace * 0.5f + disc;
  const float lam2 = trace * 0.5f - disc;

  const float eps = 1e-6f;
  return std::sqrt(lam1 / std::max(lam2, eps));
}

float computeMedian(std::vector<float> v)
{
  if (v.empty()) return 0.f;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  if (v.size() % 2 == 1) return v[mid];
  // Even: average of two middle elements
  const float hi = v[mid];
  std::nth_element(v.begin(), v.begin() + mid - 1, v.begin() + mid);
  return 0.5f * (v[mid - 1] + hi);
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Algorithm 2: Merge Tree Collapse Segmentation
// ─────────────────────────────────────────────────────────────────────────────
RoomGraph segmentRooms(
  const PersistenceResult & pr,
  const jps_planner_3d::VoxelMap & vmap,
  float tau,
  int max_portals_per_edge)
{
  auto t0 = std::chrono::steady_clock::now();

  const auto & graph    = vmap.getSurfaceGraph();
  const auto & all_nbrs = vmap.getAllNeighbors();
  const int n  = static_cast<int>(graph.size());
  const int nm = static_cast<int>(pr.nodes.size());

  RoomGraph rg;
  if (n == 0 || nm == 0) return rg;

  const double res = vmap.getResolution();

  // ── Phase 1: Identify significant maxima ──────────────────────────────────
  // M = { m ∈ nodes : pers(m) > τ }
  // The root always has pers = kInfPers, so it is always significant.
  std::vector<int> sig_nodes;  // indices into pr.nodes[]
  sig_nodes.reserve(64);
  for (int i = 0; i < nm; ++i) {
    if (pr.nodes[i].persistence > tau) {
      sig_nodes.push_back(i);
    }
  }
  const int R = static_cast<int>(sig_nodes.size());
  rg.num_rooms = R;

  // Map from pr.nodes index → dense room_id [0, R)
  std::vector<int> node_to_room(nm, -1);
  for (int r = 0; r < R; ++r) {
    node_to_room[sig_nodes[r]] = r;
  }

  // ── Phase 2: Collapse tree (top-down BFS from root) ───────────────────────
  // room_of[node_idx] = the significant ancestor's node_idx (not yet remapped)
  std::vector<int> room_of(nm, -1);

  // BFS queue — process parents before children
  std::vector<int> bfs;
  bfs.reserve(nm);
  bfs.push_back(pr.root_node);

  for (int qi = 0; qi < static_cast<int>(bfs.size()); ++qi) {
    const int m = bfs[qi];
    if (node_to_room[m] >= 0) {
      room_of[m] = m;  // significant: own room
    } else {
      // non-significant: inherit parent's room assignment
      const int par = pr.nodes[m].parent_node;
      // par must already be processed (BFS top-down guarantee)
      room_of[m] = room_of[par];
    }
    for (int child : pr.nodes[m].children) {
      bfs.push_back(child);
    }
  }

  // ── Phase 3: Label all voxels ─────────────────────────────────────────────
  rg.room_labels.assign(n, -1);
  for (int s = 0; s < n; ++s) {
    const int raw = pr.raw_labels[s];
    if (raw < 0) continue;  // should not happen
    const int room_node = room_of[raw];
    rg.room_labels[s] = node_to_room[room_node];
  }

  // ── Phase 4: Compute room attributes ─────────────────────────────────────
  // Accumulate per-room statistics
  struct RoomAcc {
    double sum_wx = 0, sum_wy = 0, sum_wz = 0;
    double sum_d  = 0;
    std::vector<float> zs;  // z values for median & variance
    std::vector<float> xs, ys; // world (x,y) for PCA
    int cnt = 0;
  };
  std::vector<RoomAcc> acc(R);

  for (int s = 0; s < n; ++s) {
    const int rid = rg.room_labels[s];
    if (rid < 0 || rid >= R) continue;
    const auto & nd = graph[s];
    auto [wx, wy, wz] = vmap.gridToWorld(nd.x, nd.y, nd.z);
    auto & a = acc[rid];
    a.sum_wx += wx; a.sum_wy += wy; a.sum_wz += wz;
    a.sum_d  += nd.distance;
    a.zs.push_back(static_cast<float>(wz));
    a.xs.push_back(static_cast<float>(wx));
    a.ys.push_back(static_cast<float>(wy));
    ++a.cnt;
  }

  rg.rooms.resize(R);
  for (int r = 0; r < R; ++r) {
    const auto & a   = acc[r];
    const int nv     = a.cnt;
    const int ni     = sig_nodes[r];
    auto & rm        = rg.rooms[r];

    rm.room_id       = r;
    rm.birth_max_surf = pr.nodes[ni].surf_idx;
    rm.birth_d       = pr.nodes[ni].birth;
    rm.num_voxels    = nv;

    if (nv > 0) {
      rm.cx = static_cast<float>(a.sum_wx / nv);
      rm.cy = static_cast<float>(a.sum_wy / nv);
      rm.cz = static_cast<float>(a.sum_wz / nv);
      rm.mean_clearance = static_cast<float>(a.sum_d / nv);
      rm.median_z = computeMedian(a.zs);

      // Height variance
      float sum_dz2 = 0;
      for (float z : a.zs) {
        const float dz = z - rm.median_z;
        sum_dz2 += dz * dz;
      }
      rm.height_var = sum_dz2 / nv;

      rm.pca_ratio = computePcaRatio(a.xs, a.ys);
    }
  }

  // ── Phase 5: Extract room adjacency and boundary attributes ───────────────
  // Collect all boundary voxels per (edge, side) for multi-portal selection.
  const int K = std::max(1, max_portals_per_edge);

  struct BndVoxel { int surf_idx; float d_value; };
  struct BndInfo {
    float min_d_i = 1e9f;
    float min_d_j = 1e9f;
    int argmax_surf_i = -1;
    int argmax_surf_j = -1;
    float max_d_i = -1.f;
    float max_d_j = -1.f;
    int len = 0;
    std::vector<BndVoxel> side_i;  // boundary voxels on room_i side
    std::vector<BndVoxel> side_j;  // boundary voxels on room_j side
  };

  // Use a flat map: key = i * R + j (i < j)
  std::unordered_map<int, BndInfo> bnd_map;
  bnd_map.reserve(R * 4);

  for (int s = 0; s < n; ++s) {
    const int ri = rg.room_labels[s];
    if (ri < 0) continue;
    const auto & nd = graph[s];
    const float ds  = static_cast<float>(nd.distance);

    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int q  = all_nbrs[nd.neighbor_start + ni];
      const int rj = rg.room_labels[q];
      if (rj < 0 || rj == ri) continue;

      // s is in B_{ri, rj}
      const int ki = std::min(ri, rj);
      const int kj = std::max(ri, rj);
      const int key = ki * R + kj;

      auto & b = bnd_map[key];
      b.len++;

      if (ri < rj) {
        b.min_d_i = std::min(b.min_d_i, ds);
        if (ds > b.max_d_i) { b.max_d_i = ds; b.argmax_surf_i = s; }
        if (K > 1) b.side_i.push_back({s, ds});
      } else {
        b.min_d_j = std::min(b.min_d_j, ds);
        if (ds > b.max_d_j) { b.max_d_j = ds; b.argmax_surf_j = s; }
        if (K > 1) b.side_j.push_back({s, ds});
      }
    }
  }

  // Select top-K portals via farthest-point sampling seeded from argmax-D.
  // Ensures geometric spread while preferring high-clearance boundary voxels.
  auto selectPortals = [&](std::vector<BndVoxel> & voxels, int seed_surf) -> std::vector<int> {
    if (voxels.empty()) return seed_surf >= 0 ? std::vector<int>{seed_surf} : std::vector<int>{};
    if (K <= 1) return {seed_surf};

    // Deduplicate (same voxel may appear multiple times from different neighbor pairs)
    std::sort(voxels.begin(), voxels.end(),
              [](const BndVoxel & a, const BndVoxel & b) { return a.surf_idx < b.surf_idx; });
    voxels.erase(std::unique(voxels.begin(), voxels.end(),
                 [](const BndVoxel & a, const BndVoxel & b) { return a.surf_idx == b.surf_idx; }),
                 voxels.end());

    if (static_cast<int>(voxels.size()) <= K) {
      std::vector<int> result;
      // Put seed first
      result.push_back(seed_surf);
      for (auto & v : voxels)
        if (v.surf_idx != seed_surf) result.push_back(v.surf_idx);
      return result;
    }

    // Farthest-point sampling from seed
    std::vector<int> selected;
    selected.push_back(seed_surf);
    std::vector<float> min_dist_sq(voxels.size(), std::numeric_limits<float>::max());

    for (int k = 1; k < K; ++k) {
      // Update min_dist from last selected portal
      const int last = selected.back();
      const auto & lv = graph[last];
      for (size_t i = 0; i < voxels.size(); ++i) {
        const auto & vi = graph[voxels[i].surf_idx];
        float dx = static_cast<float>(vi.x - lv.x);
        float dy = static_cast<float>(vi.y - lv.y);
        float dz = static_cast<float>(vi.z - lv.z);
        float d2 = dx * dx + dy * dy + dz * dz;
        min_dist_sq[i] = std::min(min_dist_sq[i], d2);
      }
      // Select farthest unselected voxel
      int best = -1;
      float best_dist = -1.f;
      for (size_t i = 0; i < voxels.size(); ++i) {
        bool already = false;
        for (int s : selected) { if (voxels[i].surf_idx == s) { already = true; break; } }
        if (already) continue;
        if (min_dist_sq[i] > best_dist) {
          best_dist = min_dist_sq[i];
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) selected.push_back(voxels[best].surf_idx);
    }
    return selected;
  };

  // Build edges from boundary map
  rg.edges.reserve(bnd_map.size());
  for (auto & [key, b] : bnd_map) {
    const int ri = key / R;
    const int rj = key % R;

    const float min_d   = std::min(b.min_d_i, b.min_d_j);
    const float pass_w  = 2.f * min_d * static_cast<float>(res);

    // Primary portal: argmax D per side
    int si = b.argmax_surf_i;
    int sj = b.argmax_surf_j;
    if (si < 0) si = (sj >= 0) ? sj : 0;
    if (sj < 0) sj = si;

    auto [wxi, wyi, wzi] = vmap.gridToWorld(graph[si].x, graph[si].y, graph[si].z);
    auto [wxj, wyj, wzj] = vmap.gridToWorld(graph[sj].x, graph[sj].y, graph[sj].z);

    RoomEdge edge;
    edge.room_i           = ri;
    edge.room_j           = rj;
    edge.passage_width_m  = pass_w;
    edge.transition_surf_i = si;
    edge.transition_surf_j = sj;
    edge.trans_wx         = 0.5f * (static_cast<float>(wxi) + static_cast<float>(wxj));
    edge.trans_wy         = 0.5f * (static_cast<float>(wyi) + static_cast<float>(wyj));
    edge.trans_wz         = 0.5f * (static_cast<float>(wzi) + static_cast<float>(wzj));
    edge.boundary_length  = b.len / 2;

    // Multi-portal selection
    if (K > 1) {
      edge.transitions_i = selectPortals(b.side_i, si);
      edge.transitions_j = selectPortals(b.side_j, sj);
    } else {
      edge.transitions_i = {si};
      edge.transitions_j = {sj};
    }

    const int eidx = static_cast<int>(rg.edges.size());
    rg.edges.push_back(edge);

    rg.adjacency[ri].emplace_back(rj, eidx);
    rg.adjacency[rj].emplace_back(ri, eidx);
  }

  auto t1 = std::chrono::steady_clock::now();
  rg.segmentation_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  return rg;
}

}  // namespace topo_nav
