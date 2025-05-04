#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

def load_odom(path):
    """
    Betölt egy odom-csv-t. Ha nincs benne 'x','y' oszlop, 
    akkor header=None, kézi oszlopnevekkel újratölti.
    """
    df = pd.read_csv(path, skipinitialspace=True)
    if 'x' not in df.columns or 'y' not in df.columns:
        # nincs fejléc, vegyük fel mi magunk
        names = ['sec','nanosec','x','y','z','qx','qy','qz','qw']
        df = pd.read_csv(path, header=None, names=names, skipinitialspace=True)
    return df

def main(real_path, sim_path):
    if not os.path.exists(real_path) or not os.path.exists(sim_path):
        print("Hiba: egyik fájl sem található:", real_path, sim_path, file=sys.stderr)
        sys.exit(1)

    real = load_odom(real_path)
    sim  = load_odom(sim_path)

    real_x, real_y = real['x'].to_numpy(), real['y'].to_numpy()
    sim_x,  sim_y  = sim['x'].to_numpy(),  sim['y'].to_numpy()

    plt.figure(figsize=(8,8))
    plt.plot(real_x, real_y,  label='Valódi odom',     linestyle='-',  marker='.', alpha=0.8, color='tab:blue')
    plt.plot(sim_x,  sim_y,   label='Szimulált odom', linestyle='--', marker='x', alpha=0.8, color='tab:orange')
    plt.axis('equal')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Valódi vs. Szimulált Odom Trajektória')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # fájlok útvonala
    real_csv = '/home/ezo/f1tenth_ws/src/data_logger/logged_file/filtered/odom.csv'
    sim_csv  = '/home/ezo/f1tenth_ws/src/data_logger/logged_file/simulated/sim_odom.csv'
    main(real_csv, sim_csv)
