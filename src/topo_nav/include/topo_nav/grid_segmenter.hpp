#pragma once

#include "topo_nav/room_segmenter.hpp"   // RoomGraph, RoomNode, RoomEdge, RoomType

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

/// Grid-based room segmentation (Config C / baseline).
///
/// Partitions the surface graph into uniform 3D grid cells, then builds
/// the same RoomGraph structure (rooms, edges, adjacency, room_labels)
/// used by the hierarchical planner.
///
/// @param vmap               VoxelMap with ready surface graph
/// @param target_room_count  Desired number of rooms (0 = auto-size based on scene extent)
/// @param tau                Persistence tau (only used for auto room count estimation)
/// @returns RoomGraph compatible with detectFloors() and setTopoMap()
RoomGraph segmentRoomsGrid(
  const jps_planner_3d::VoxelMap & vmap,
  int target_room_count = 0,
  float tau = 3.0f);

}  // namespace topo_nav
