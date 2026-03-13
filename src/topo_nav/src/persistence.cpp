#include "topo_nav/persistence.hpp"
#include "jps_planner_3d/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <vector>

namespace topo_nav
{

// ─────────────────────────────────────────────────────────────────────────────
// Union-Find with path-halving and explicit union-by-birth policy.
//
// "Birth" = D value of the local maximum that is the root of each component.
// Only local maxima are ever roots; non-maximum nodes are attached directly
// to the current root of the component they join.
// ─────────────────────────────────────────────────────────────────────────────
namespace
{

struct UnionFind
{
  std::vector<int> parent;  // -1 = not yet in UF
  std::vector<int> birth_d; // D value at root; only valid when parent[i]==i

  explicit UnionFind(int n) : parent(n, -1), birth_d(n, -1) {}

  /// Create a singleton component for local maximum x with D value d.
  void makeSet(int x, int d)
  {
    parent[x]  = x;
    birth_d[x] = d;
  }

  /// Attach non-maximum node x directly to root r.
  void attach(int x, int r) { parent[x] = r; }

  /// Find root with path halving (safe only on nodes already in UF).
  int find(int x)
  {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];  // path halving
      x = parent[x];
    }
    return x;
  }

  /// Merge two root components.  winner (higher birth_d) stays as root.
  /// Caller guarantees that loser and winner are already roots.
  void uniteKeepRoot(int loser, int winner)
  {
    // loser and winner are roots, so parent[x] == x for both.
    parent[loser] = winner;
  }
};

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Algorithm 1: Persistence Computation with Merge Tree Construction
// ─────────────────────────────────────────────────────────────────────────────
PersistenceResult computePersistence(const jps_planner_3d::VoxelMap & vmap)
{
  auto t0 = std::chrono::steady_clock::now();

  const auto & graph    = vmap.getSurfaceGraph();
  const auto & all_nbrs = vmap.getAllNeighbors();
  const int n = static_cast<int>(graph.size());

  PersistenceResult result;
  result.raw_labels.assign(n, -1);

  if (n == 0) {
    return result;
  }

  // ── Step 1: Sort surface nodes by (D desc, x asc, y asc, z asc) ──────────
  // This implements the total order ≺ from the paper so that each flat plateau
  // has exactly one local maximum (ties broken lexicographically).
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
    [&graph](int a, int b) {
      const auto & na = graph[a];
      const auto & nb = graph[b];
      if (na.distance != nb.distance) return na.distance > nb.distance;
      if (na.x != nb.x)               return na.x < nb.x;
      if (na.y != nb.y)               return na.y < nb.y;
      return na.z < nb.z;
    });

  // ── Step 2: Sweep ─────────────────────────────────────────────────────────
  UnionFind uf(n);
  std::vector<bool> processed(n, false);

  // Temporary buffer for distinct roots of processed neighbours.
  // In practice a room has at most 4 cardinal neighbours, so this is tiny.
  std::vector<int> roots;
  roots.reserve(8);

  for (int k = 0; k < n; ++k) {
    const int s = order[k];
    const auto & node = graph[s];

    // Collect distinct UF-roots of already-processed neighbours.
    roots.clear();
    for (int ni = 0; ni < node.neighbor_count; ++ni) {
      const int q = all_nbrs[node.neighbor_start + ni];
      if (!processed[q]) continue;
      const int r = uf.find(q);
      bool already = false;
      for (int rr : roots) {
        if (rr == r) { already = true; break; }
      }
      if (!already) roots.push_back(r);
    }

    if (roots.empty()) {
      // ── Case 1: new local maximum (leaf in merge tree) ──────────────────
      uf.makeSet(s, node.distance);
      const int node_idx = static_cast<int>(result.nodes.size());
      result.nodes.push_back({
        s,
        static_cast<float>(node.distance),
        -1.f,     // death: set when merged
        kInfPers, // persistence: overwritten on merge (root keeps kInfPers)
        -1,       // parent_node
        -1,       // merge_surf_idx
        {}        // children
      });
      result.surf_to_node[s] = node_idx;
      result.raw_labels[s]   = node_idx;

    } else if (roots.size() == 1) {
      // ── Case 2: s joins an existing component ────────────────────────────
      const int c = roots[0];
      uf.attach(s, c);
      result.raw_labels[s] = result.surf_to_node[c];

    } else {
      // ── Case 3: saddle point — merge K ≥ 2 components ───────────────────
      // Sort roots by birth_d descending: c1 (highest birth) survives.
      std::sort(roots.begin(), roots.end(),
        [&uf](int a, int b) {
          return uf.birth_d[a] > uf.birth_d[b];
        });
      const int c1      = roots[0];
      const int node_c1 = result.surf_to_node[c1];

      for (int j = 1; j < static_cast<int>(roots.size()); ++j) {
        const int cj      = roots[j];
        const int node_cj = result.surf_to_node[cj];

        // Fill in merge-tree edge: cj is absorbed by c1 at saddle s.
        result.nodes[node_cj].death         = static_cast<float>(node.distance);
        result.nodes[node_cj].persistence   =
          static_cast<float>(uf.birth_d[cj] - node.distance);
        result.nodes[node_cj].parent_node   = node_c1;
        result.nodes[node_cj].merge_surf_idx = s;
        result.nodes[node_c1].children.push_back(node_cj);

        uf.uniteKeepRoot(cj, c1);
      }

      // Attach the saddle point itself to c1.
      uf.attach(s, c1);
      result.raw_labels[s] = node_c1;
    }

    processed[s] = true;
  }

  // ── Step 3: Identify root of merge tree ───────────────────────────────────
  // The root is the global maximum — the one node with no parent.
  // All other nodes were absorbed at some point.
  for (int i = 0; i < static_cast<int>(result.nodes.size()); ++i) {
    if (result.nodes[i].parent_node < 0) {
      result.root_node = i;
      result.nodes[i].persistence = kInfPers;
      result.nodes[i].death       = -1.f;
      break;
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  result.compute_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  return result;
}

}  // namespace topo_nav
