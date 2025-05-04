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

def load_tf(path, parent='odom', child='base_link'):
    """
    Betölt egy tf-log CSV-t, kiszűri a parent→child transzformokat,
    és timestamp-et generál.
    """
    df = pd.read_csv(path, skipinitialspace=True)
    # ellenőrizzük a kötelező oszlopokat
    for c in ['sec','nanosec','frame_id','child_frame_id','tx','ty','tz','qx','qy','qz','qw']:
        if c not in df.columns:
            raise RuntimeError(f"{path} nem tartalmazza a(z) {c} oszlopot")
    df = df[(df.frame_id==parent) & (df.child_frame_id==child)].copy()
    df['timestamp'] = df.sec + df.nanosec * 1e-9
    return df.sort_values('timestamp').reset_index(drop=True)

def plot_tf_trajectory(real_tf, sim_tf):
    """
    Origóba igazítja és kirajzolja a real vs sim TF pályát.
    """
    # Origin-align: vonjuk le az első pontot
    x0r, y0r = real_tf.loc[0, ['tx','ty']]
    x0s, y0s = sim_tf.loc[0,  ['tx','ty']]
    real_tf['x0'] = real_tf.tx - x0r
    real_tf['y0'] = real_tf.ty - y0r
    sim_tf ['x0'] = sim_tf.tx  - x0s
    sim_tf ['y0'] = sim_tf.ty  - y0s

    # Tartományok
    min_x = min(real_tf.x0.min(), sim_tf.x0.min())
    max_x = max(real_tf.x0.max(), sim_tf.x0.max())
    min_y = min(real_tf.y0.min(), sim_tf.y0.min())
    max_y = max(real_tf.y0.max(), sim_tf.y0.max())

    print(f"TF Real X: {min_x:.3f} … {max_x:.3f}, Y: {min_y:.3f} … {max_y:.3f}")
    print(f"TF Sim  X: {min_x:.3f} … {max_x:.3f}, Y: {min_y:.3f} … {max_y:.3f}")

    plt.figure(figsize=(6,6))
    plt.plot(real_tf['x0'].to_numpy(), real_tf['y0'].to_numpy(),
             '-',  label='Valódi TF-pálya', alpha=0.8)
    plt.plot(sim_tf['x0'].to_numpy(),  sim_tf['y0'].to_numpy(),
             '--', label='Szimulált TF-pálya', alpha=0.8)

    plt.xlim(min_x - 0.1, max_x + 0.1)
    plt.ylim(min_y - 0.1, max_y + 0.1)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X [m]'); plt.ylabel('Y [m]')
    plt.title('Valódi vs Szimulált TF Trajektória')
    plt.legend(); plt.grid(True); plt.tight_layout()

    out_png = os.path.join(BASE_DIR, 'tf_trajectory_aligned.png')
    plt.savefig(out_png)
    print(f"TF pálya plot mentve ide: {out_png}")

def compute_rmse_tf(real_tf, sim_tf):
    dx = sim_tf.x0 - real_tf.x0
    dy = sim_tf.y0 - real_tf.y0
    return np.sqrt(np.mean(dx**2 + dy**2))

def main():
    # CSV fájlok
    real_tf_path = os.path.join(BASE_DIR, 'filtered',  'tf.csv')
    sim_tf_path  = os.path.join(BASE_DIR, 'simulated','sim_tf.csv')

    # Betöltés
    real_tf = load_tf(real_tf_path, parent='odom', child='base_link')
    sim_tf  = load_tf(sim_tf_path,  parent='odom', child='base_link')

    print(f"Real TF loaded: {len(real_tf)} sor")
    print(f"Sim  TF loaded: {len(sim_tf)} sor")

    # Plot és RMSE
    plot_tf_trajectory(real_tf, sim_tf)
    rmse = compute_rmse_tf(real_tf, sim_tf)
    print(f"TF-alapú pozíció RMSE: {rmse:.3f} m")

if __name__ == '__main__':
    main()
