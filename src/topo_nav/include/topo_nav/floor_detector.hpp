#pragma once

#include "topo_nav/room_segmenter.hpp"

#include <vector>

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

struct FloorNode
{
  int floor_id;
  float height_min, height_max;  ///< Min/max room-median heights (world, metres)
  int num_rooms;
  int num_voxels;
};

struct FloorEdge
{
  int floor_i, floor_j;   ///< floor_i < floor_j
  int staircase_room_id;  ///< Room id of the staircase connecting them
  float traversal_cost;   ///< |height_max_j - height_min_i| (rough estimate)
};

/// Floor-level topological graph (Level 2 of the hierarchy).
struct FloorGraph
{
  std::vector<FloorNode> floors;
  std::vector<FloorEdge> edges;

  /// adjacency[floor_id] = list of {neighbor_floor_id, edge_idx}
  std::vector<std::vector<std::pair<int, int>>> adjacency;
};

/// Detect floors, classify rooms, and build the floor graph.
///
/// Modifies rg.rooms in-place: fills floor_id and type fields.
///
/// @param rg             Room graph (modified in place)
/// @param vmap           VoxelMap (for resolution)
/// @param H_floor_m      Floor-separation gap threshold (metres, default 2.5)
/// @param sigma_stair_m  Staircase height std-dev threshold (metres, default 0.5)
/// @param gamma          Corridor PCA aspect-ratio threshold (default 3.0)
FloorGraph detectFloors(
  RoomGraph & rg,
  const jps_planner_3d::VoxelMap & vmap,
  float H_floor_m     = 2.5f,
  float sigma_stair_m = 0.5f,
  float gamma         = 3.0f);

}  // namespace topo_nav
