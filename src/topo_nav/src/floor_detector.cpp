#include "topo_nav/floor_detector.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace topo_nav
{

FloorGraph detectFloors(
  RoomGraph & rg,
  const jps_planner_3d::VoxelMap & /*vmap*/,
  float H_floor_m,
  float sigma_stair_m,
  float gamma)
{
  FloorGraph fg;
  const int R = rg.num_rooms;
  if (R == 0) return fg;

  // ── Step 1: Classify staircase rooms (before floor assignment) ─────────────
  // Priority: staircase check first (may also be elongated but height variance
  // is the defining characteristic).
  const float stair_var_thresh = sigma_stair_m * sigma_stair_m;

  for (int r = 0; r < R; ++r) {
    auto & rm = rg.rooms[r];
    if (rm.height_var > stair_var_thresh) {
      rm.type = RoomType::STAIRCASE;
    } else if (rm.pca_ratio > gamma) {
      rm.type = RoomType::CORRIDOR;
    } else {
      rm.type = RoomType::ROOM;
    }
  }

  // ── Step 2: Floor clustering via 1-D gap detection on median_z ────────────
  // Sort rooms by median z
  std::vector<int> order(R);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
    [&rg](int a, int b) {
      return rg.rooms[a].median_z < rg.rooms[b].median_z;
    });

  // Gap detection: consecutive median-z values that differ by ≥ H_floor_m
  // start a new floor.
  int current_floor = 0;
  for (int k = 0; k < R; ++k) {
    if (k > 0) {
      const float gap =
        rg.rooms[order[k]].median_z - rg.rooms[order[k - 1]].median_z;
      if (gap >= H_floor_m) {
        ++current_floor;
      }
    }
    rg.rooms[order[k]].floor_id = current_floor;
  }
  const int F = current_floor + 1;

  // ── Step 3: Build FloorNode list ───────────────────────────────────────────
  fg.floors.resize(F);
  for (int f = 0; f < F; ++f) {
    fg.floors[f].floor_id      = f;
    fg.floors[f].height_min    =  1e9f;
    fg.floors[f].height_max    = -1e9f;
    fg.floors[f].num_rooms     = 0;
    fg.floors[f].num_voxels    = 0;
  }

  for (int r = 0; r < R; ++r) {
    const auto & rm = rg.rooms[r];
    auto & fl = fg.floors[rm.floor_id];
    fl.height_min = std::min(fl.height_min, rm.median_z);
    fl.height_max = std::max(fl.height_max, rm.median_z);
    ++fl.num_rooms;
    fl.num_voxels += rm.num_voxels;
  }

  // ── Step 4: Build FloorGraph edges from staircase rooms ───────────────────
  fg.adjacency.resize(F);

  for (int r = 0; r < R; ++r) {
    if (rg.rooms[r].type != RoomType::STAIRCASE) continue;
    const int f_self = rg.rooms[r].floor_id;

    // A staircase room connects its floor to the floors of its adjacent rooms.
    const auto it = rg.adjacency.find(r);
    if (it == rg.adjacency.end()) continue;

    for (auto & [nbr_room, eidx] : it->second) {
      const int f_nbr = rg.rooms[nbr_room].floor_id;
      if (f_nbr == f_self) continue;

      const int fi = std::min(f_self, f_nbr);
      const int fj = std::max(f_self, f_nbr);

      // Avoid duplicate floor edges
      bool dup = false;
      for (auto & [nf, neidx] : fg.adjacency[fi]) {
        if (nf == fj && fg.edges[neidx].staircase_room_id == r) {
          dup = true; break;
        }
      }
      if (dup) continue;

      FloorEdge fe;
      fe.floor_i          = fi;
      fe.floor_j          = fj;
      fe.staircase_room_id = r;
      fe.traversal_cost   =
        std::abs(fg.floors[fj].height_max - fg.floors[fi].height_min);

      const int feidx = static_cast<int>(fg.edges.size());
      fg.edges.push_back(fe);
      fg.adjacency[fi].emplace_back(fj, feidx);
      fg.adjacency[fj].emplace_back(fi, feidx);
    }
  }

  return fg;
}

}  // namespace topo_nav
