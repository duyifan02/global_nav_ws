#pragma once

#include <vector>
#include <unordered_map>
#include <limits>
#include <string>

// Forward-declare VoxelMap to avoid including the heavyweight header here.
namespace jps_planner_3d { class VoxelMap; }

namespace topo_nav
{

/// A node in the merge tree — one per local maximum of D.
///
/// The tree is rooted at the global maximum. Edges run from absorbed child to
/// absorbing parent (child.parent_node = parent's index in nodes[]).
struct MergeTreeNode
{
  int surf_idx;       ///< Index in VoxelMap surface graph
  float birth;        ///< D(surf_idx) — value at birth
  float death;        ///< D at merge saddle; -1 for root
  float persistence;  ///< birth – death; kInfPers for root
  int parent_node;    ///< Index into PersistenceResult::nodes; -1 for root
  int merge_surf_idx; ///< Saddle surface-graph index; -1 for root
  std::vector<int> children; ///< Indices into PersistenceResult::nodes
};

static constexpr float kInfPers = std::numeric_limits<float>::infinity();

/// Output of computePersistence() — Algorithm 1 from the paper.
struct PersistenceResult
{
  /// All local maxima of D, stored as merge-tree nodes.
  /// nodes[root_node].parent_node == -1.
  std::vector<MergeTreeNode> nodes;

  /// raw_labels[surf_idx] = index into nodes[] of the birth maximum of the
  /// component that first reached surf_idx during the decreasing sweep.
  std::vector<int> raw_labels;

  /// Maps surface-graph index of a local maximum → its index in nodes[].
  std::unordered_map<int, int> surf_to_node;

  /// Index of the global maximum (root of the merge tree).
  int root_node = -1;

  double compute_time_ms = 0.0;
};

/// Compute 0-dimensional persistent homology on D (Algorithm 1).
///
/// Sweeps D from max to 0 using a Union-Find, builds the merge tree, and
/// records raw labels for every surface point.  Time O(|S| log |S|).
PersistenceResult computePersistence(const jps_planner_3d::VoxelMap & vmap);

}  // namespace topo_nav
