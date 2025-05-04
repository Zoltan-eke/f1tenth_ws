#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import matplotlib
# Agg backend fájlba mentéshez (nincs GUI)
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# BASE_DIR a data_logger/logged_file mappája
SCRIPT_DIR = os.path.dirname(__file__)
BASE_DIR   = os.path.abspath(os.path.join(
    SCRIPT_DIR, '..', 'logged_file'))

def load_and_timestamp(path):
    df = pd.read_csv(path, skipinitialspace=True)
    if 'sec' in df.columns and 'nanosec' in df.columns:
        df['timestamp'] = df.sec + df.nanosec * 1e-9
    else:
        raise RuntimeError(f"{path} nem tartalmaz sec/nanosec oszlopokat")
    return df.sort_values('timestamp').reset_index(drop=True)

def merge_series(drive, odom):
    return pd.merge_asof(
        drive, odom,
        on='timestamp',
        direction='nearest',
        suffixes=('_cmd','_odom')
    )

def plot_trajectory(real, sim):
    # x0,y0 mezők használata (origin-aligned)
    real_x = real['x0'].to_numpy()
    real_y = real['y0'].to_numpy()
    sim_x  = sim ['x0'].to_numpy()
    sim_y  = sim ['y0'].to_numpy()

    # Mindkét pálya tartományának kiszámítása
    min_x = min(real_x.min(), sim_x.min())
    max_x = max(real_x.max(), sim_x.max())
    min_y = min(real_y.min(), sim_y.min())
    max_y = max(real_y.max(), sim_y.max())

    print(f"Real X: {min_x:.3f} … {max_x:.3f}, Real Y: {min_y:.3f} … {max_y:.3f}")
    print(f"Sim  X: {min_x:.3f} … {max_x:.3f}, Sim  Y: {min_y:.3f} … {max_y:.3f}")

    plt.figure(figsize=(6,6))
    plt.plot(real_x, real_y, '-',  label='Valódi odom',    alpha=0.8)
    plt.plot(sim_x,  sim_y,  '--', label='Szimulált odom', alpha=0.8)

    # Explicit tengelyhatárok beállítása, hogy ne vágjon le semmit
    plt.xlim(min_x - 0.1, max_x + 0.1)
    plt.ylim(min_y - 0.1, max_y + 0.1)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Valódi vs. Szimulált Odom Trajektória')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    out_png = os.path.join(BASE_DIR, 'trajectory_aligned.png')
    plt.savefig(out_png)
    print(f"Pálya plot mentve ide: {out_png}")

def compute_rmse(real, sim):
    dx = sim['x0'] - real['x0']
    dy = sim['y0'] - real['y0']
    return np.sqrt(np.mean(dx**2 + dy**2))

def main():
    # fájlok
    real_drive = os.path.join(BASE_DIR, 'filtered',  'drive.csv')
    real_odom  = os.path.join(BASE_DIR, 'filtered',  'odom.csv')
    sim_drive  = os.path.join(BASE_DIR, 'simulated', 'sim_drive.csv')
    sim_odom   = os.path.join(BASE_DIR, 'simulated', 'sim_odom.csv')

    # betöltés
    rd = load_and_timestamp(real_drive)
    ro = load_and_timestamp(real_odom)
    sd = load_and_timestamp(sim_drive)
    so = load_and_timestamp(sim_odom)

    print(f"Real drive: {len(rd)} sor, real odom: {len(ro)} sor")
    print(f"Sim drive: {len(sd)} sor,  sim odom: {len(so)} sor")

    # sync
    real = merge_series(rd, ro)
    sim  = merge_series(sd, so)

    # átnevezés
    real.rename(columns={'x':'x_odom','y':'y_odom'}, inplace=True)
    sim .rename(columns={'x':'x_odom','y':'y_odom'}, inplace=True)

    # origin-align
    x0_r, y0_r = real .loc[0, ['x_odom','y_odom']]
    x0_s, y0_s = sim  .loc[0, ['x_odom','y_odom']]
    real['x0'] = real['x_odom'] - x0_r
    real['y0'] = real['y_odom'] - y0_r
    sim ['x0'] = sim ['x_odom'] - x0_s
    sim ['y0'] = sim ['y_odom'] - y0_s

    # plot + rmse
    plot_trajectory(real, sim)
    rmse = compute_rmse(real, sim)
    print(f'Pozíció RMSE (aligned): {rmse:.3f} m')

if __name__ == '__main__':
    main()
