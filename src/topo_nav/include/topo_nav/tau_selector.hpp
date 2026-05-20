#pragma once

#include "topo_nav/persistence.hpp"
#include "topo_nav/room_segmenter.hpp"

#include <vector>

namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

/// Per-τ lightweight statistics (no precompute, just room segmentation).
struct TauStats
{
  float tau                  = 0.0f;
  int   num_rooms            = 0;
  int   num_edges            = 0;
  int   num_portals          = 0;   ///< 2 × num_edges (portal nodes in TG)
  int   tg_connected_est     = 0;   ///< Estimated TG components (from adjacency)
  int   max_room_voxels      = 0;
  int   max_room_portals     = 0;
  double max_room_fraction   = 0.0; ///< max_room_voxels / |V_surface|
  double room_size_gini      = 0.0; ///< Gini coefficient of room sizes
  int   mean_portals_per_room = 0;
  double max_budget_ratio    = 0.0; ///< max(k_i * |R_i|) / budget
  int   budget_skip_rooms    = 0;   ///< Rooms that would be skipped by budget
  double skip_voxel_fraction = 0.0; ///< Fraction of voxels in would-be-skipped rooms
  double coverage_est        = 0.0; ///< Estimated transfer coverage (1.0 = no skips)
  double score               = 0.0; ///< Composite selector score (higher = better)
};

/// Result of τ selection.
struct TauChoice
{
  float selected_tau = 5.0f;
  int   selected_idx = -1;          ///< Index into candidates
  std::vector<TauStats> candidates;
  double selection_time_ms = 0.0;
};

/// Evaluate candidate τ values and select the best one.
///
/// Runs segmentRooms() for each candidate (cheap: just merge-tree collapse)
/// and scores them by a rule-based composite metric. No precompute needed.
///
/// @param pr         Result of computePersistence() (reused across all τ)
/// @param vmap       VoxelMap
/// @param candidates τ values to evaluate (default: {3,4,5,6})
TauChoice selectTau(
  const PersistenceResult & pr,
  const jps_planner_3d::VoxelMap & vmap,
  const std::vector<float> & candidates = {3.0f, 4.0f, 5.0f, 6.0f});

}  // namespace topo_nav
