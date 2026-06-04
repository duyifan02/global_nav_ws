#!/usr/bin/env python3
"""Detailed per-scene analysis to understand speedup distribution and outliers."""
import csv
import os
from collections import defaultdict

RESULTS_DIR = "experiments/results/hm3d_100"

def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))

def scene_stats(rows):
    by_scene = defaultdict(list)
    for r in rows:
        scene = r["scene"]
        ok = r["success"] == "True"
        ts = float(r["planning_time_ms"]) if ok else None
        pl = float(r["path_length_m"]) if ok and r.get("path_length_m") else None
        fb = int(r["ns_visited_nodes"]) if ok and r.get("ns_visited_nodes") else 0
        topo = float(r["topo_ms"]) if r.get("topo_ms") and r["topo_ms"] else 0
        by_scene[scene].append({"ok": ok, "ts": ts, "pl": pl, "fb": fb, "topo": topo})
    out = {}
    for scene, trials in by_scene.items():
        ok_t = [t for t in trials if t["ok"]]
        n_ok = len(ok_t)
        n = len(trials)
        mean_ts = sum(t["ts"] for t in ok_t) / n_ok if n_ok else None
        mean_pl = sum(t["pl"] for t in ok_t if t["pl"]) / max(1, len([t for t in ok_t if t["pl"]])) if n_ok else None
        max_fb = max((t["fb"] for t in ok_t), default=0)
        total_fb = sum(t["fb"] for t in ok_t)
        mean_topo = sum(t["topo"] for t in ok_t) / n_ok if n_ok else 0
        out[scene] = dict(n_ok=n_ok, n_total=n, mean_ts=mean_ts,
                          mean_pl=mean_pl, max_fb=max_fb, total_fb=total_fb,
                          mean_topo=mean_topo)
    return out

def main():
    os.chdir("/data2/dyf/papers/Beyond_Geometry/global_nav_ws")

    alt  = scene_stats(load_csv(f"{RESULTS_DIR}/100_alt_01m.csv"))
    tg5  = scene_stats(load_csv(f"{RESULTS_DIR}/100_tg_tau5_01m.csv"))
    tg3  = scene_stats(load_csv(f"{RESULTS_DIR}/100_tg_tau3_01m.csv"))
    full = scene_stats(load_csv(f"{RESULTS_DIR}/100_full_tau5_01m.csv"))
    jps  = scene_stats(load_csv(f"{RESULTS_DIR}/100_jps_01m.csv"))

    all_scenes = sorted(alt.keys())

    # Build per-scene comparison rows
    rows = []
    for scene in all_scenes:
        a = alt.get(scene)
        t5 = tg5.get(scene)
        t3 = tg3.get(scene)
        fu = full.get(scene)
        j = jps.get(scene)
        if not a or not t5:
            continue
        a_ts  = a["mean_ts"]
        t5_ts = t5["mean_ts"]
        t3_ts = t3["mean_ts"] if t3 else None
        fu_ts = fu["mean_ts"] if fu else None
        j_ts  = j["mean_ts"] if j else None

        if a_ts and t5_ts:
            speedup = a_ts / t5_ts
        else:
            speedup = None

        rows.append({
            "scene": scene.replace("hm3d_", ""),
            "jps_ts": j_ts,
            "alt_ts": a_ts,
            "tg5_ts": t5_ts,
            "tg3_ts": t3_ts,
            "fu_ts": fu_ts,
            "speedup_vs_alt": speedup,
            "tg5_fb": t5["max_fb"] if t5 else 0,
            "tg3_fb": t3["max_fb"] if t3 else 0,
            "tg5_ok": f"{t5['n_ok']}/{t5['n_total']}" if t5 else "-",
            "a_pl": a["mean_pl"],
            "t5_pl": t5["mean_pl"],
        })

    # Sort by speedup
    rows_sorted = sorted([r for r in rows if r["speedup_vs_alt"]],
                         key=lambda x: x["speedup_vs_alt"], reverse=True)

    print("=" * 110)
    print("  All 100 scenes — ranked by PH-ALT-TG τ=5 speedup vs ALT (descending)")
    print("=" * 110)
    print(f"\n{'#':>3} {'Scene':<28} {'JPS':>7} {'flat':>7} {'ALT':>7} {'TG τ=5':>8} {'TG τ=3':>8} {'full':>7} {'Speedup':>8} {'MaxFB':>8} {'OK':>6}")
    print("-" * 110)

    flat_ss = scene_stats(load_csv(f"{RESULTS_DIR}/100_flat_01m.csv"))
    for i, r in enumerate(rows_sorted, 1):
        scene = r["scene"]
        flat_ts = flat_ss.get(f"hm3d_{scene}", {}).get("mean_ts") or flat_ss.get(scene, {}).get("mean_ts")
        jts = f"{r['jps_ts']:.3f}" if r["jps_ts"] else "FAIL"
        fts = f"{flat_ts:.3f}" if flat_ts else "FAIL"
        ats = f"{r['alt_ts']:.3f}" if r["alt_ts"] else "FAIL"
        t5ts = f"{r['tg5_ts']:.3f}" if r["tg5_ts"] else "FAIL"
        t3ts = f"{r['tg3_ts']:.3f}" if r["tg3_ts"] else "FAIL"
        futs = f"{r['fu_ts']:.3f}" if r["fu_ts"] else "FAIL"
        sp = f"{r['speedup_vs_alt']:.1f}x" if r["speedup_vs_alt"] else "-"
        fb = r["tg5_fb"] if r["tg5_fb"] else 0
        marker = " ★" if r["speedup_vs_alt"] and r["speedup_vs_alt"] >= 5.0 else ""
        marker2 = " ▼" if r["speedup_vs_alt"] and r["speedup_vs_alt"] < 0.5 else ""
        print(f"{i:>3} {scene:<28} {jts:>7} {fts:>7} {ats:>7} {t5ts:>8} {t3ts:>8} {futs:>7} {sp:>8} {fb:>8} {r['tg5_ok']:>6}{marker}{marker2}")

    # Summary stats
    speedups = [r["speedup_vs_alt"] for r in rows_sorted if r["speedup_vs_alt"]]
    n_total = len(speedups)
    n_beats = sum(1 for s in speedups if s > 1.0)
    n_far_worse = sum(1 for s in speedups if s < 0.5)
    n_worse = sum(1 for s in speedups if s < 1.0)

    speedups_sorted = sorted(speedups)
    median = speedups_sorted[len(speedups_sorted)//2]
    p25 = speedups_sorted[len(speedups_sorted)//4]
    p75 = speedups_sorted[3*len(speedups_sorted)//4]

    print(f"\n{'='*110}")
    print(f"  Summary Statistics: PH-ALT-TG τ=5 vs ALT A*")
    print(f"{'='*110}")
    print(f"  N scenes compared: {n_total}")
    print(f"  Beats ALT:     {n_beats}/{n_total} ({n_beats/n_total*100:.0f}%)")
    print(f"  Worse than ALT:{n_worse}/{n_total} ({n_worse/n_total*100:.0f}%)")
    print(f"  Much worse (<0.5x): {n_far_worse}/{n_total}")
    print(f"  Speedup percentiles — p25: {p25:.2f}x, median: {median:.2f}x, p75: {p75:.2f}x")
    print(f"  Mean speedup (all scenes): {sum(speedups)/len(speedups):.2f}x")
    print(f"  Mean speedup (scenes where TG wins): {sum(s for s in speedups if s>1.0)/max(1,n_beats):.2f}x")

    # Investigate slow cases: scenes where TG is much slower
    print(f"\n{'='*110}")
    print(f"  Slow cases: scenes where TG τ=5 is >2x slower than ALT (speedup < 0.5x)")
    print(f"{'='*110}")
    print(f"\n  {'Scene':<28} {'ALT':>7} {'TG τ=5':>8} {'TG τ=3':>8} {'MaxFB τ=5':>11} {'MaxFB τ=3':>11}")
    print("  " + "-" * 80)
    for r in rows_sorted[::-1]:
        if r["speedup_vs_alt"] and r["speedup_vs_alt"] < 0.5:
            alt_ts = f"{r['alt_ts']:.3f}" if r['alt_ts'] else "-"
            t5_ts = f"{r['tg5_ts']:.3f}" if r['tg5_ts'] else "-"
            t3_ts = f"{r['tg3_ts']:.3f}" if r['tg3_ts'] else "-"
            fb5 = r["tg5_fb"] if r["tg5_fb"] else 0
            t3_data = tg3.get(f"hm3d_{r['scene']}", tg3.get(r['scene'], {}))
            fb3 = t3_data.get("max_fb", 0)
            print(f"  {r['scene']:<28} {alt_ts:>7} {t5_ts:>8} {t3_ts:>8} {fb5:>11,} {fb3:>11,}")

    # Compare TG τ=3 vs τ=5 vs ALT — which is best per scene
    print(f"\n{'='*110}")
    print(f"  Best method per scene (τ=3 vs τ=5 vs ALT)")
    print(f"{'='*110}")
    best_tg5 = best_tg3 = best_alt = 0
    for r in rows_sorted:
        ts_list = [(r["tg5_ts"], "TG τ=5"), (r["tg3_ts"], "TG τ=3"), (r["alt_ts"], "ALT")]
        valid = [(t, n) for t, n in ts_list if t is not None]
        if not valid:
            continue
        best = min(valid, key=lambda x: x[0])
        if best[1] == "TG τ=5": best_tg5 += 1
        elif best[1] == "TG τ=3": best_tg3 += 1
        else: best_alt += 1
    n = best_tg5 + best_tg3 + best_alt
    print(f"  TG τ=5 fastest:  {best_tg5}/{n} ({best_tg5/n*100:.0f}%)")
    print(f"  TG τ=3 fastest:  {best_tg3}/{n} ({best_tg3/n*100:.0f}%)")
    print(f"  ALT fastest:     {best_alt}/{n} ({best_alt/n*100:.0f}%)")

    # Also check if combining: "TG τ=5 OR τ=3 beats ALT"
    both_beat = sum(1 for r in rows_sorted
                    if r["speedup_vs_alt"] and r["speedup_vs_alt"] > 1.0)
    either_beat = sum(1 for r in rows_sorted
                      if (r["tg5_ts"] and r["alt_ts"] and r["tg5_ts"] < r["alt_ts"])
                      or (r["tg3_ts"] and r["alt_ts"] and r["tg3_ts"] < r["alt_ts"]))
    print(f"  Either TG (τ=3 or τ=5) beats ALT: {either_beat}/{n} ({either_beat/n*100:.0f}%)")

if __name__ == "__main__":
    main()
