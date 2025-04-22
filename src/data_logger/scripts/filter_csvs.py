#!/usr/bin/env python3
"""
Filter CSV files by method: manual time window or IMU jerk threshold.
Saves filtered files into a 'filtered' subdirectory next to the 'raw' directory.
Usage:
  # Manual window:
  python3 filter_csvs.py --method manual --t-min 48.0 --t-max 132.0

  # Automatic IMU-based jerk detection:
  python3 filter_csvs.py --method imu --j-threshold 0.2
"""
import argparse
from pathlib import Path
import pandas as pd
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(
        description='Filter logger CSV files by time window or IMU jerk detection'
    )
    parser.add_argument(
        '--input-dir', '-i',
        type=Path,
        default=Path.home() / 'f1tenth_ws' / 'src' / 'data_logger' / 'logged_file' / 'raw',
        help='Directory containing raw CSV files'
    )
    parser.add_argument(
        '--method',
        choices=['manual', 'imu'],
        required=True,
        help='Filtering method: manual time window or IMU jerk threshold'
    )
    parser.add_argument(
        '--t-min',
        type=float,
        help='Manual start time (s) relative to first timestamp'
    )
    parser.add_argument(
        '--t-max',
        type=float,
        help='Manual stop time (s) relative to first timestamp'
    )
    parser.add_argument(
        '--j-threshold',
        type=float,
        help='Jerk threshold (m/s^3) for IMU-based detection'
    )
    args = parser.parse_args()
    if args.method == 'manual' and (args.t_min is None or args.t_max is None):
        parser.error('Manual method requires --t-min and --t-max')
    if args.method == 'imu' and args.j_threshold is None:
        parser.error('IMU method requires --j-threshold')
    return args


def detect_time_window_imu(imu_csv: Path, j_threshold: float):
    df = pd.read_csv(imu_csv)
    # Absolute time in seconds
    df['time'] = df['sec'] + df['nanosec'] * 1e-9
    # Compute acceleration magnitude minus gravity
    df['acc_mag'] = np.sqrt(df['ax']**2 + df['ay']**2 + df['az']**2) - 9.81
    # Jerk = derivative of acceleration
    dt_vals = df['time'].diff().fillna(method='bfill')
    df['jerk'] = df['acc_mag'].diff().fillna(0) / dt_vals
    # Indices where jerk exceeds threshold
    idx = df.index[np.abs(df['jerk']) > j_threshold]
    if idx.empty:
        # Fallback to full range
        return df['time'].iloc[0], df['time'].iloc[-1]
    return df['time'].iloc[idx[0]], df['time'].iloc[idx[-1]]


def filter_csv(file_path: Path, output_dir: Path, t0: float, t1: float) -> int:
    df = pd.read_csv(file_path)
    if 'sec' not in df.columns or 'nanosec' not in df.columns:
        print(f"[!] Skipping {file_path.name}: no timestamp columns.")
        return 0
    # Compute absolute time
    df['time'] = df['sec'] + df['nanosec'] * 1e-9
    # Filter within window
    df_filtered = df[(df['time'] >= t0) & (df['time'] <= t1)]
    if df_filtered.empty:
        print(f"[!] Warning: no data in {file_path.name} between {t0:.3f}s and {t1:.3f}s.")
        print(f"    Original range: {df['time'].min():.3f}s–{df['time'].max():.3f}s")
    # Determine edge tolerance based on sampling interval
    dt_vals = df['time'].diff().dropna().values
    if len(dt_vals):
        median_dt = np.median(dt_vals)
        edge_tol = median_dt * 2
    else:
        edge_tol = 1e-3  # default 1 ms
    # Check for data near edges
    near_start = df[(df['time'] >= t0 - edge_tol) & (df['time'] < t0 + edge_tol)]
    near_stop  = df[(df['time'] > t1 - edge_tol)  & (df['time'] <= t1 + edge_tol)]
    if near_start.empty or near_stop.empty:
        print(f"[!] Warning: no data near window edges in {file_path.name} (tol={edge_tol:.3f}s)")
    # Save filtered CSV
    out_path = output_dir / f'filtered_{file_path.name}'
    df_filtered.to_csv(out_path, index=False)
    return len(df_filtered)


def main():
    args = parse_args()
    raw_dir = args.input_dir
    filtered_dir = raw_dir.parent / 'filtered'
    filtered_dir.mkdir(exist_ok=True)

    if args.method == 'manual':
        # Compute absolute window from odom.csv reference
        ref = pd.read_csv(raw_dir / 'odom.csv')
        base = ref['sec'].iloc[0] + ref['nanosec'].iloc[0] * 1e-9
        t0_abs = base + args.t_min
        t1_abs = base + args.t_max
        print(f"Manual window: {args.t_min}s–{args.t_max}s -> {t0_abs:.3f}s–{t1_abs:.3f}s")
    else:
        t0_abs, t1_abs = detect_time_window_imu(raw_dir / 'imu.csv', args.j_threshold)
        print(f"IMU window: {t0_abs:.3f}s–{t1_abs:.3f}s (jerk_th={args.j_threshold})")

    # Process each CSV
    for fname in ['odom.csv', 'imu.csv', 'joint_states.csv', 'drive.csv']:
        path = raw_dir / fname
        if not path.exists():
            print(f"[!] Missing file: {path}")
            continue
        count = filter_csv(path, filtered_dir, t0_abs, t1_abs)
        print(f"[✓] {fname}: {count} rows -> {filtered_dir / ('filtered_' + fname)}")

if __name__ == '__main__':
    main()
