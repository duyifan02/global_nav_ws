#!/usr/bin/env python3
"""Analyze 100-scene HM3D benchmark results (6 methods, start-as-seed, 0.1m)."""
import csv
import os
from collections import defaultdict

RESULTS_DIR = "experiments/results/hm3d_100"

METHODS = {
    "100_jps_01m.csv":        "JPS (Surface A*)",
    "100_flat_01m.csv":       "flat_surface",
    "100_alt_01m.csv":        "ALT A* (16 lm)",
    "100_tg_tau5_01m.csv":    "PH-ALT-TG τ=5",
    "100_tg_tau3_01m.csv":    "PH-ALT-TG τ=3",
    "100_full_tau5_01m.csv":  "full (HPA*) τ=5",
}

def load_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows

def scene_stats(rows):
    """Per-scene: {scene: {n_ok, n_total, mean_ts, mean_pl, max_fb, mean_topo}}"""
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
        max_fb = max((t["fb"] for t in ok_t), default=None)
        mean_topo = sum(t["topo"] for t in ok_t) / n_ok if n_ok else 0
        out[scene] = dict(n_ok=n_ok, n_total=n, mean_ts=mean_ts,
                          mean_pl=mean_pl, max_fb=max_fb, mean_topo=mean_topo)
    return out

def overall(ss):
    ok = {s: v for s, v in ss.items() if v["mean_ts"] is not None}
    if not ok:
        return None, None, 0, len(ss)
    ts = sum(v["mean_ts"] for v in ok.values()) / len(ok)
    pl_vals = [v["mean_pl"] for v in ok.values() if v["mean_pl"]]
    pl = sum(pl_vals) / len(pl_vals) if pl_vals else None
    n_ok = sum(v["n_ok"] for v in ss.values())
    n_total = sum(v["n_total"] for v in ss.values())
    return ts, pl, n_ok, n_total

def main():
    data = {}
    for fname, label in METHODS.items():
        path = os.path.join(RESULTS_DIR, fname)
        rows = load_csv(path)
        ss = scene_stats(rows)
        ts, pl, n_ok, n_total = overall(ss)
        data[label] = dict(ss=ss, ts=ts, pl=pl, n_ok=n_ok, n_total=n_total)

    jps_ts  = data["JPS (Surface A*)"]["ts"]
    flat_ts = data["flat_surface"]["ts"]
    alt_ts  = data["ALT A* (16 lm)"]["ts"]
    alt_pl  = data["ALT A* (16 lm)"]["pl"]

    print("=" * 95)
    print("  100-Scene HM3D Benchmark — 0.1m, start-as-seed, 3 episodes/scene")
    print("=" * 95)
    print(f"\n{'Method':<22} {'Ts(ms)':>8} {'Path(m)':>8} {'OK':>8} {'vs JPS':>8} {'vs flat':>8} {'vs ALT':>8} {'Path%':>8}")
    print("-" * 95)

    order = ["JPS (Surface A*)", "flat_surface", "ALT A* (16 lm)",
             "full (HPA*) τ=5", "PH-ALT-TG τ=3", "PH-ALT-TG τ=5"]
    for label in order:
        d = data[label]
        ts, pl = d["ts"], d["pl"]
        ok = f"{d['n_ok']}/{d['n_total']}"
        if ts:
            vj = f"{jps_ts/ts:.1f}x"
            vf = f"{flat_ts/ts:.1f}x"
            va = f"{alt_ts/ts:.1f}x"
            ph = f"{(pl/alt_pl-1)*100:+.1f}%" if pl and alt_pl else "-"
            print(f"{label:<22} {ts:>8.3f} {pl:>8.1f} {ok:>8} {vj:>8} {vf:>8} {va:>8} {ph:>8}")
        else:
            print(f"{label:<22} {'FAIL':>8} {'-':>8} {ok:>8}")

    # Scene-level success breakdown
    print(f"\n{'='*95}")
    print("  Scene Success Rates (n_ok / n_total per method)")
    print(f"{'='*95}")

    all_scenes = sorted(data["JPS (Surface A*)"]["ss"].keys())
    short_methods = ["JPS (Surface A*)", "flat_surface", "ALT A* (16 lm)",
                     "PH-ALT-TG τ=5", "PH-ALT-TG τ=3"]
    n_fail_any = 0
    n_fail_all = 0
    n_scenes = len(all_scenes)

    fail_scenes = []
    for scene in all_scenes:
        row = []
        any_ok = False
        all_ok = True
        for m in short_methods:
            ss_m = data[m]["ss"].get(scene)
            if ss_m:
                row.append(f"{ss_m['n_ok']}/{ss_m['n_total']}")
                if ss_m["n_ok"] > 0:
                    any_ok = True
                if ss_m["n_ok"] < ss_m["n_total"]:
                    all_ok = False
            else:
                row.append("?/?")
        if not any_ok:
            n_fail_all += 1
            fail_scenes.append(scene)
        if not all_ok and any_ok:
            n_fail_any += 1

    print(f"\n  {n_scenes} total scenes, start-as-seed")
    print(f"  Scenes with ALL methods failing: {n_fail_all}")
    if fail_scenes:
        for s in fail_scenes[:5]:
            print(f"    - {s}")
        if len(fail_scenes) > 5:
            print(f"    ... and {len(fail_scenes)-5} more")
    print(f"  Scenes with partial failures: {n_fail_any}")

    # Distribution of query times for best method
    print(f"\n{'='*95}")
    print("  PH-ALT-TG τ=5 — Per-Scene Mean Query Time Distribution")
    print(f"{'='*95}")
    tg5 = data["PH-ALT-TG τ=5"]["ss"]
    alt_ss = data["ALT A* (16 lm)"]["ss"]
    pairs = []
    for scene in all_scenes:
        t5 = tg5.get(scene)
        ta = alt_ss.get(scene)
        if t5 and t5["mean_ts"] and ta and ta["mean_ts"]:
            pairs.append((scene, t5["mean_ts"], ta["mean_ts"], t5["mean_ts"]/ta["mean_ts"]))

    # Speedup distribution
    if pairs:
        speedups = [p[3] for p in pairs]
        speedups_vs_alt = [alt_ts_s/ts_s for _, ts_s, alt_ts_s, _ in pairs]
        beats = sum(1 for s in speedups_vs_alt if s > 1.0)
        print(f"\n  Scenes where TG τ=5 beats ALT: {beats}/{len(pairs)}")
        print(f"  Speedup vs ALT — min: {min(speedups_vs_alt):.2f}x, median: {sorted(speedups_vs_alt)[len(speedups_vs_alt)//2]:.2f}x, max: {max(speedups_vs_alt):.2f}x")
        print(f"  Speedup bins vs ALT:")
        bins = [(0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 5.0), (5.0, 10.0), (10.0, float("inf"))]
        for lo, hi in bins:
            n = sum(1 for s in speedups_vs_alt if lo <= s < hi)
            label_str = f"[{lo:.0f}x, {hi:.0f}x)" if hi < float("inf") else f"[{lo:.0f}x, ∞)"
            bar = "█" * n
            print(f"    {label_str:>14}: {n:3d} scenes  {bar}")

    # Top speedup scenes
    print(f"\n  Top 10 scenes by speedup (TG τ=5 vs ALT):")
    print(f"  {'Scene':<30} {'TG τ=5 (ms)':>12} {'ALT (ms)':>10} {'Speedup':>10} {'OK':>6}")
    for scene, ts_s, alt_ts_s, _ in sorted(pairs, key=lambda x: x[2]/x[1], reverse=True)[:10]:
        sp = alt_ts_s / ts_s
        t5_ok = tg5[scene]["n_ok"]
        t5_n = tg5[scene]["n_total"]
        print(f"  {scene.replace('hm3d_',''):<30} {ts_s:>12.3f} {alt_ts_s:>10.3f} {sp:>10.1f}x {t5_ok}/{t5_n}")

    # Overall comparison with precompute
    print(f"\n{'='*95}")
    print("  Amortized Cost (precompute + N queries)")
    print(f"{'='*95}")
    # Mean topo_ms per method
    for label in ["ALT A* (16 lm)", "PH-ALT-TG τ=5", "PH-ALT-TG τ=3", "full (HPA*) τ=5"]:
        d = data[label]
        ok_ss = [v for v in d["ss"].values() if v["mean_ts"]]
        mean_topo = sum(v["mean_topo"] for v in ok_ss) / len(ok_ss) if ok_ss else 0
        print(f"  {label:<22}: precompute={mean_topo:.0f}ms, query={d['ts']:.3f}ms")

if __name__ == "__main__":
    os.chdir("/data2/dyf/papers/Beyond_Geometry/global_nav_ws")
    main()
