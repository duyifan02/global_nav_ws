#!/usr/bin/env python3
"""
PH-ALT-TG parameter sweep: tau, landmark count, and resolution.

Runs auto_benchmark.py with different configurations, collecting results
into separate CSVs. Designed for overnight autonomous execution.

Usage:
    python3 sweep_ph_alt_tg.py [--experiments tau,landmarks,res05]
"""
import subprocess
import sys
import shutil
import argparse
from pathlib import Path
from datetime import datetime

SCRIPT_DIR = Path(__file__).parent
AUTO_BENCH = SCRIPT_DIR / "auto_benchmark.py"
RESULTS_DIR = Path(__file__).resolve().parent.parent.parent.parent / "experiments" / "results"
WS_DIR = Path(__file__).resolve().parent.parent.parent.parent

# Input CSV templates
INPUT_01M = WS_DIR / "benchmark_v4_input.csv"


def run_benchmark(output_csv: Path, tau: float, num_landmarks: int,
                  trials: int, resolution: float = 0.1,
                  input_csv: Path = None, planner_mode: str = "ph_alt_tg",
                  extra_args: list = None):
    """Run auto_benchmark.py with given parameters."""
    if input_csv is None:
        input_csv = INPUT_01M

    # Copy input CSV to output location (auto_benchmark writes results in-place)
    shutil.copy2(input_csv, output_csv)

    cmd = [
        sys.executable, str(AUTO_BENCH),
        str(output_csv),
        "--planner", "TopoNav",
        "--planner-mode", planner_mode,
        "--tau", str(tau),
        "--num-landmarks", str(num_landmarks),
        "--trials", str(trials),
        "--force",
        "--service-timeout", "30.0",
        "--map-load-timeout", "300.0",
    ]
    if extra_args:
        cmd.extend(extra_args)

    print(f"\n{'='*70}")
    print(f"[SWEEP] tau={tau}, landmarks={num_landmarks}, trials={trials}, res={resolution}")
    print(f"[SWEEP] output: {output_csv.name}")
    print(f"[SWEEP] cmd: {' '.join(cmd)}")
    print(f"{'='*70}\n")

    result = subprocess.run(cmd, capture_output=False, text=True, timeout=1800)
    if result.returncode != 0:
        print(f"[WARN] Benchmark returned code {result.returncode}")
    return result.returncode


def create_005m_input(output_path: Path):
    """Create 0.05m input CSV from 0.1m template by changing resolution."""
    import csv
    with open(INPUT_01M, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames

    for row in rows:
        row['resolution'] = '0.05'
        # Clear result columns
        for col in ['planning_time_ms', 'success', 'planning_time_std_ms',
                     'path_length_m', 'ns_visited_nodes', 'tp_ms', 'te_ms',
                     'topo_ms', 'memory_usage_bytes', 'total_time_ms', 'trials_count']:
            if col in row:
                row[col] = ''

    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[SWEEP] Created 0.05m input: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="PH-ALT-TG parameter sweep")
    parser.add_argument('--experiments', type=str, default='tau,landmarks,res05',
                        help='Comma-separated experiments: tau, landmarks, res05')
    parser.add_argument('--trials', type=int, default=5,
                        help='Number of trials per configuration')
    args = parser.parse_args()

    experiments = [e.strip() for e in args.experiments.split(',')]
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # ── Experiment 1: Tau sweep at 0.1m ──────────────────────────────────────
    if 'tau' in experiments:
        print("\n" + "="*70)
        print("EXPERIMENT 1: Tau Sweep (0.1m, 8 landmarks)")
        print("="*70)

        for tau in [1, 2, 3, 4, 5, 8]:
            out_csv = RESULTS_DIR / f"ph_alt_tg_tau{tau}_01m_results.csv"
            try:
                run_benchmark(out_csv, tau=tau, num_landmarks=8,
                              trials=args.trials, resolution=0.1)
            except subprocess.TimeoutExpired:
                print(f"[ERROR] Timeout for tau={tau}")
            except Exception as e:
                print(f"[ERROR] tau={tau}: {e}")

    # ── Experiment 2: Landmark ablation at 0.1m ─────────────────────────────
    if 'landmarks' in experiments:
        print("\n" + "="*70)
        print("EXPERIMENT 2: Landmark Ablation (0.1m, tau=3)")
        print("="*70)

        for lm in [0, 2, 4, 8, 16]:
            out_csv = RESULTS_DIR / f"ph_alt_tg_lm{lm}_01m_results.csv"
            try:
                run_benchmark(out_csv, tau=3, num_landmarks=lm,
                              trials=args.trials, resolution=0.1)
            except subprocess.TimeoutExpired:
                print(f"[ERROR] Timeout for landmarks={lm}")
            except Exception as e:
                print(f"[ERROR] landmarks={lm}: {e}")

    # ── Experiment 3: 0.05m resolution with higher tau ──────────────────────
    if 'res05' in experiments:
        print("\n" + "="*70)
        print("EXPERIMENT 3: 0.05m Resolution (tau=5,8, 8 landmarks)")
        print("="*70)

        input_005m = RESULTS_DIR / "ph_alt_tg_005m_input.csv"
        create_005m_input(input_005m)

        for tau in [5, 8]:
            out_csv = RESULTS_DIR / f"ph_alt_tg_tau{tau}_005m_results.csv"
            try:
                run_benchmark(out_csv, tau=tau, num_landmarks=8,
                              trials=args.trials, resolution=0.05,
                              input_csv=input_005m)
            except subprocess.TimeoutExpired:
                print(f"[ERROR] Timeout for tau={tau} at 0.05m")
            except Exception as e:
                print(f"[ERROR] tau={tau} at 0.05m: {e}")

    print("\n" + "="*70)
    print(f"[SWEEP] All experiments complete at {datetime.now()}")
    print("="*70)


if __name__ == "__main__":
    main()
