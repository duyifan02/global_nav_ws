# Graph Speedup Baselines: Feasibility Analysis

## Context

The TopoNav paper compares against ALT (A* with Landmarks and Triangle inequality)
as the primary baseline. Reviewers may ask why we didn't compare against other
classical graph speedup techniques. This note addresses feasibility.

## Surface Graph Properties

Our surface graph at 0.1m resolution has:
- |V| ~ 5k-15k voxels per scene (HM3D)
- |E| ~ 30k-100k edges (26-connected)
- Irregular structure (non-planar, variable degree)
- Edge weights are Euclidean distances (positive, ~0.1-0.17m)
- Queries require full path reconstruction (not just distance)

At 0.05m: |V| ~ 30k-80k, |E| ~ 200k-500k

## Baseline Analysis

### Contraction Hierarchies (CH)
- **Preprocessing**: O(|V| log |V|) shortcut edges. For |V|=10k this is fast (<1s).
- **Query**: Bidirectional Dijkstra on augmented graph, typically 10-100x speedup.
- **Path reconstruction**: Unpack shortcuts to original edges — adds complexity.
- **Memory**: |E| + shortcuts, typically 2-5x original edges.
- **Feasibility**: FEASIBLE. Would be a strong baseline. However:
  - Implementation complexity is significant (node ordering, witness search)
  - Preprocessing must be repeated per-scene (not amortizable across scenes)
  - Our ALT baseline is already strong (16 landmarks give excellent heuristic)
  - At |V|=10k, ALT query is already <0.2ms — CH speedup may be marginal

### Hub Labeling (2-Hop Labeling)
- **Preprocessing**: O(|V|^2) for exact, O(|V| * |R|) for pruned. At |V|=10k, memory is
  ~800MB for exact (10k * 10k * 8 bytes). Pruned labels are smaller but still large.
- **Query**: O(k) where k is label size, typically O(sqrt(|V|)).
- **Path reconstruction**: Requires storing predecessor info or re-running local search.
- **Memory**: Dominated by label storage. For |V|=80k at 0.05m, this is prohibitive.
- **Feasibility**: NOT FEASIBLE at 0.05m. Marginal at 0.1m. Memory is the bottleneck.

### Arc Flags
- **Preprocessing**: Partition graph, run backward Dijkstra from each boundary node.
  For R regions with B boundary nodes: O(B * |V| log |V|).
- **Query**: A* with arc flags as filter. Typically 10-50x speedup.
- **Path reconstruction**: Same as standard A* — straightforward.
- **Memory**: |E| * R bits for flags. At R=16, adds 2 bytes per edge.
- **Feasibility**: FEASIBLE but marginal benefit. With |V|=10k, vanilla A* is already
  sub-millisecond. Arc Flags shine on road networks with |V|>100k.

### Customizable Route Planning (CRP) / Multi-Level Dijkstra
- **Preprocessing**: Graph partitioning + boundary node extraction + overlay graph.
- **Query**: Multi-level search, similar to HPA*.
- **Feasibility**: CONCEPTUALLY SIMILAR to our method. TG is essentially a 2-level
  hierarchy. CRP uses nested partitions; we use PH-derived rooms. Comparing CRP
  vs TG is comparing partitioning strategies, not fundamentally different approaches.

## Why ALT is the Right Primary Baseline

1. **Implementation quality**: ALT is well-understood, easy to implement correctly,
   and has provably admissible heuristic.
2. **Preprocessing is comparable**: 16 Dijkstra runs (~50-80ms) vs TG construction (~50-100ms).
3. **Strongest among simple techniques**: ALT with 16 landmarks gives a tight lower
   bound that often matches the optimal distance, making the search very efficient.
4. **Path reconstruction is trivial**: Standard A* parent-chain backtracking.
5. **Applicable to irregular graphs**: Unlike CH (which assumes planarity helps) or
   Hub Labeling (which needs small separator structure), ALT works on any graph.

## Summary

| Technique | Preprocess | Query | Memory | Path Recon | Feasibility |
|-----------|-----------|-------|--------|------------|-------------|
| ALT (done) | 50-80ms | 0.17ms | 16*|V|*8B | trivial | DONE |
| CH | <1s | ~0.01ms | 2-5x|E| | moderate | feasible but marginal |
| Hub Label | minutes | ~0.001ms | ~|V|^1.5 * 8B | hard | memory-limited |
| Arc Flags | seconds | ~0.05ms | |E|*R bits | trivial | marginal benefit |
| CRP | seconds | ~0.01ms | overlay graph | moderate | similar to TG |

**Recommendation**: Mention CH and Arc Flags in Related Work as established graph
speedup techniques. Note that our ALT baseline already achieves sub-millisecond
queries at 0.1m, so the speedup ceiling for any technique is bounded. Our
contribution is the room-aware hierarchical structure that enables topological
reasoning, not raw query speed.
