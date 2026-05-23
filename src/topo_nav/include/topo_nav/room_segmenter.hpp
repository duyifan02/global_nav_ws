#pragma once

#include "topo_nav/persistence.hpp"

#include <vector>
#include <unordered_map>
#include <string>

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

enum class RoomType { ROOM, CORRIDOR, STAIRCASE };

/// Geometric attributes of one detected room.
struct RoomNode
{
  int room_id;           ///< 0-based dense room index
  int birth_max_surf;    ///< Surface-graph index of the room's birth maximum
  float birth_d;         ///< D value at birth maximum (room "centre clearance")

  // 3D centroid in world coordinates
  float cx, cy, cz;

  float median_z;        ///< Median z (world) for floor detection
  float height_var;      ///< Var(z | L=i) in m² for staircase detection
  float mean_clearance;  ///< Mean D (voxel units) over all room voxels
  int num_voxels;        ///< Number of surface voxels in the room
  float pca_ratio;       ///< sqrt(λ1/λ2) of (x,y) PCA for corridor detection

  int floor_id = -1;
  RoomType type = RoomType::ROOM;
};

/// Attributes of the shared boundary between two rooms.
struct RoomEdge
{
  int room_i, room_j;    ///< Canonical order: room_i < room_j

  /// passage_width_m = 2 * min_D * resolution (metres)
  float passage_width_m;

  /// Primary portal: argmax D(s) on B_ij (room_i side of boundary)
  int transition_surf_i;
  /// Primary portal: argmax D(s) on B_ji (room_j side of boundary)
  int transition_surf_j;

  /// World-coordinate midpoint of the two primary transition voxels
  float trans_wx, trans_wy, trans_wz;

  int boundary_length;   ///< |B_ij|

  /// Multi-portal: all selected portals per side (includes primary as [0]).
  /// When max_portals_per_edge=1, these contain exactly the primary portal.
  std::vector<int> transitions_i;
  std::vector<int> transitions_j;
};

/// Room-level topological graph (Level 1 of the hierarchy).
struct RoomGraph
{
  std::vector<RoomNode> rooms;
  std::vector<RoomEdge> edges;

  /// room_labels[surf_idx] = room_id  (-1 = unassigned, shouldn't happen)
  std::vector<int> room_labels;

  /// adjacency[room_id] = list of {neighbor_room_id, edge_idx}
  std::unordered_map<int, std::vector<std::pair<int, int>>> adjacency;

  int num_rooms = 0;
  double segmentation_time_ms = 0.0;
};

/// Segment rooms via merge-tree collapse (Algorithm 2 from the paper).
///
/// @param pr   Result of computePersistence()
/// @param vmap VoxelMap (for surface graph, coordinates, distance field)
/// @param tau  Persistence threshold in voxel units (e.g. 3)
/// @param max_portals_per_edge  Max portals per side of each room edge (1 = legacy)
RoomGraph segmentRooms(
  const PersistenceResult & pr,
  const jps_planner_3d::VoxelMap & vmap,
  float tau,
  int max_portals_per_edge = 1);

}  // namespace topo_nav
