#include "topo_nav/grid_segmenter.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace topo_nav
{

namespace
{

float computeMedianGrid(std::vector<float> v)
{
  if (v.empty()) return 0.f;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  if (v.size() % 2 == 1) return v[mid];
  const float hi = v[mid];
  std::nth_element(v.begin(), v.begin() + mid - 1, v.begin() + mid);
  return 0.5f * (v[mid - 1] + hi);
}

float computePcaRatioGrid(const std::vector<float> & xs, const std::vector<float> & ys)
{
  const int n = static_cast<int>(xs.size());
  if (n < 3) return 1.0f;
  float mx = 0, my = 0;
  for (int i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; }
  mx /= n; my /= n;
  float cxx = 0, cxy = 0, cyy = 0;
  for (int i = 0; i < n; ++i) {
    const float dx = xs[i] - mx;
    const float dy = ys[i] - my;
    cxx += dx * dx; cxy += dx * dy; cyy += dy * dy;
  }
  cxx /= n; cxy /= n; cyy /= n;
  const float trace = cxx + cyy;
  const float disc  = std::sqrt(std::max(0.f,
    (cxx - cyy) * (cxx - cyy) * 0.25f + cxy * cxy));
  const float lam1 = trace * 0.5f + disc;
  const float lam2 = trace * 0.5f - disc;
  return std::sqrt(lam1 / std::max(lam2, 1e-6f));
}

/// Hash for (cx, cy, cz) cell coords.
struct CellHash {
  size_t operator()(const std::tuple<int,int,int>& c) const {
    auto h1 = std::hash<int>{}(std::get<0>(c));
    auto h2 = std::hash<int>{}(std::get<1>(c));
    auto h3 = std::hash<int>{}(std::get<2>(c));
    return h1 ^ (h2 * 2654435761u) ^ (h3 * 40503u);
  }
};

}  // anonymous namespace

RoomGraph segmentRoomsGrid(
  const jps_planner_3d::VoxelMap & vmap,
  int target_room_count,
  float tau)
{
  auto t0 = std::chrono::steady_clock::now();

  const auto & graph    = vmap.getSurfaceGraph();
  const auto & all_nbrs = vmap.getAllNeighbors();
  const int n = static_cast<int>(graph.size());
  const double res = vmap.getResolution();

  RoomGraph rg;
  if (n == 0) return rg;

  // Compute bounding box in grid coords
  int gx_min = graph[0].x, gx_max = gx_min;
  int gy_min = graph[0].y, gy_max = gy_min;
  int gz_min = graph[0].z, gz_max = gz_min;
  for (int s = 1; s < n; ++s) {
    const auto & nd = graph[s];
    gx_min = std::min(gx_min, nd.x); gx_max = std::max(gx_max, nd.x);
    gy_min = std::min(gy_min, nd.y); gy_max = std::max(gy_max, nd.y);
    gz_min = std::min(gz_min, nd.z); gz_max = std::max(gz_max, nd.z);
  }

  // Auto-size cells: aim for ~target rooms, or use a heuristic
  if (target_room_count <= 0) {
    // Estimate: use tau-based heuristic — about 1 room per (2*tau)^3 volume
    const double cell_vol = std::pow(2.0 * tau, 3.0);
    const double total_vol = static_cast<double>(
      (gx_max - gx_min + 1)) * (gy_max - gy_min + 1) * (gz_max - gz_min + 1);
    target_room_count = std::max(2, static_cast<int>(total_vol / cell_vol));
  }

  const double volume = static_cast<double>(
    gx_max - gx_min + 1) * (gy_max - gy_min + 1) * (gz_max - gz_min + 1);
  const double cell_side = std::cbrt(volume / std::max(1, target_room_count));
  const int cell_size = std::max(1, static_cast<int>(std::round(cell_side)));

  // Assign each surface voxel to a grid cell
  std::unordered_map<std::tuple<int,int,int>, int, CellHash> cell_to_room;
  cell_to_room.reserve(target_room_count * 2);
  rg.room_labels.assign(n, -1);
  int next_room = 0;

  for (int s = 0; s < n; ++s) {
    const auto & nd = graph[s];
    const int cx = (nd.x - gx_min) / cell_size;
    const int cy = (nd.y - gy_min) / cell_size;
    const int cz = (nd.z - gz_min) / cell_size;
    auto key = std::make_tuple(cx, cy, cz);

    auto it = cell_to_room.find(key);
    if (it == cell_to_room.end()) {
      cell_to_room[key] = next_room;
      rg.room_labels[s] = next_room;
      ++next_room;
    } else {
      rg.room_labels[s] = it->second;
    }
  }

  const int R = next_room;
  rg.num_rooms = R;

  // ── Compute room attributes ──────────────────────────────────────────────
  struct RoomAcc {
    double sum_wx = 0, sum_wy = 0, sum_wz = 0, sum_d = 0;
    std::vector<float> zs, xs, ys;
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
    const auto & a = acc[r];
    const int nv = a.cnt;
    auto & rm = rg.rooms[r];
    rm.room_id = r;
    rm.birth_max_surf = -1;  // no PH birth for grid rooms
    rm.birth_d = 0.f;
    rm.num_voxels = nv;
    if (nv > 0) {
      rm.cx = static_cast<float>(a.sum_wx / nv);
      rm.cy = static_cast<float>(a.sum_wy / nv);
      rm.cz = static_cast<float>(a.sum_wz / nv);
      rm.mean_clearance = static_cast<float>(a.sum_d / nv);
      rm.median_z = computeMedianGrid(const_cast<std::vector<float>&>(a.zs));
      float sum_dz2 = 0;
      for (float z : a.zs) {
        const float dz = z - rm.median_z;
        sum_dz2 += dz * dz;
      }
      rm.height_var = sum_dz2 / nv;
      rm.pca_ratio = computePcaRatioGrid(a.xs, a.ys);
    }
  }

  // ── Extract boundary edges (same logic as room_segmenter.cpp Phase 5) ────
  struct BndInfo {
    float max_d_i = -1.f, max_d_j = -1.f;
    int argmax_surf_i = -1, argmax_surf_j = -1;
    float min_d_i = 1e9f, min_d_j = 1e9f;
    int len = 0;
  };

  std::unordered_map<int64_t, BndInfo> bnd_map;
  bnd_map.reserve(R * 4);

  for (int s = 0; s < n; ++s) {
    const int ri = rg.room_labels[s];
    if (ri < 0) continue;
    const auto & nd = graph[s];
    const float ds = static_cast<float>(nd.distance);

    for (int ni = 0; ni < nd.neighbor_count; ++ni) {
      const int q  = all_nbrs[nd.neighbor_start + ni];
      const int rj = rg.room_labels[q];
      if (rj < 0 || rj == ri) continue;

      const int ki = std::min(ri, rj);
      const int kj = std::max(ri, rj);
      const int64_t key = static_cast<int64_t>(ki) * R + kj;

      auto & b = bnd_map[key];
      b.len++;

      if (ri < rj) {
        b.min_d_i = std::min(b.min_d_i, ds);
        if (ds > b.max_d_i) { b.max_d_i = ds; b.argmax_surf_i = s; }
      } else {
        b.min_d_j = std::min(b.min_d_j, ds);
        if (ds > b.max_d_j) { b.max_d_j = ds; b.argmax_surf_j = s; }
      }
    }
  }

  rg.edges.reserve(bnd_map.size());
  for (auto & [key, b] : bnd_map) {
    const int ri = static_cast<int>(key / R);
    const int rj = static_cast<int>(key % R);

    const float min_d  = std::min(b.min_d_i, b.min_d_j);
    const float pass_w = 2.f * min_d * static_cast<float>(res);

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
    edge.trans_wx = 0.5f * (static_cast<float>(wxi) + static_cast<float>(wxj));
    edge.trans_wy = 0.5f * (static_cast<float>(wyi) + static_cast<float>(wyj));
    edge.trans_wz = 0.5f * (static_cast<float>(wzi) + static_cast<float>(wzj));
    edge.boundary_length = b.len / 2;

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
