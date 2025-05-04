#!/usr/bin/env python3
"""
offline_validation.py

Offline validáció pipeline:
  1) Beolvassa a sim_drive.csv és sim_odom_filt.csv fájlokat
  2) Kiszámolja a yaw-t a quaternionből, ha szükséges
  3) Közös, 50 Hz-es időrácsra interpolálja/adaptálja az adatokat
  4) Lefuttatja a dinamikus bicikli-modellt (RK4 integráció),
     a valódi odom-pozícióból számított sebességgel (Fx bevonva)
  5) Számolja a pozíciós és yaw hibákat, RMSE-t, max hibát (yaw wrap)
  6) Kirajzolja a trajektória-összevetést és hiba-görbéket
"""

import argparse
import yaml
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- Default járműparaméterek (felülírja a YAML) ---
LF = 0.15  # első tengely–CG távolság [m]
LR = 0.17  # hátsó tengely–CG távolság [m]

# Parancssori argumentumok
parser = argparse.ArgumentParser(
    description='Offline validáció pipeline – elfogad LF, LR paramétereket YAML‑ből'
)
parser.add_argument('--params','-p', help='YAML fájl útvonala', required=False)
parser.add_argument('--no-plot', action='store_true', help='Ne mutassa az ábrákat')
args = parser.parse_args()

# YAML paraméterek beolvasása
cfg = {}
if args.params:
    cfg = yaml.safe_load(open(args.params, 'r')).get('ros__parameters', {})
# geometriai
LF   = float(cfg.get('wheelbase_front', LF))
LR   = float(cfg.get('wheelbase_rear',  LR))
# dinamikai
m     = float(cfg.get('mass',   1.0))
Iz    = float(cfg.get('I_z',    1.0))
B     = float(cfg.get('B',      0.0))
C     = float(cfg.get('C',      0.0))
D     = float(cfg.get('D',      0.0))
C_lin = float(cfg.get('C_lin',  0.0))
print(f"[offline_validation] LF={LF}, LR={LR}, m={m}, Iz={Iz}")
print(f"[offline_validation] B={B}, C={C}, D={D}, C_lin={C_lin}")

if args.no_plot:
    import matplotlib; matplotlib.use('Agg')

# --- TESTRESZABÁS: útvonalak ---
WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
SIM_DIR        = os.path.join(WORKSPACE_ROOT,'src/data_logger/logged_file/simulated')
DRIVE_CSV = os.path.join(SIM_DIR, 'sim_drive.csv')
ODOM_CSV  = os.path.join(SIM_DIR, 'sim_odom_filt.csv')

def wrap_diff(a, b):
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))

def dynamics(state, delta, dt, v_real_prev=None, v_real=None):
    """
    Dinamikus bicikli‐modell, egyszerűsített Pacejka + lineáris gumi
    + hajtó/fékezőerő.
    state = [v_x, v_y, psi_dot, x, y, yaw]
    """
    v_x, v_y, psi_dot, x, y, yaw = state

    # 1) Longitudinális erő (Fx)
    Fx = 0.0
    if v_real_prev is not None and v_real is not None:
        Fx = m * (v_real - v_real_prev) / dt

    # 2) slip‐szögek
#    alpha_f = delta - (v_y + LF * psi_dot) / max(v_x, 1e-3)
#    alpha_r =    - (v_y - LR * psi_dot) / max(v_x, 1e-3)
    alpha_f = delta - (v_y + LF*psi_dot) / max(v_x, 1e-3)
    alpha_r =    - (v_y - LR*psi_dot) / max(v_x, 1e-3)
    # numerikus clamp: ±30° körül már nyílegyenes erő sem kell
    max_alpha = np.deg2rad(30.0)
    alpha_f = np.clip(alpha_f, -max_alpha, max_alpha)
    alpha_r = np.clip(alpha_r, -max_alpha, max_alpha)
    # 3) Pacejka + lineáris
    F_yf_nl = D * np.sin(C * np.arctan(B * alpha_f))
    F_yr_nl = D * np.sin(C * np.arctan(B * alpha_r))
    F_yf_li = C_lin * alpha_f
    F_yr_li = C_lin * alpha_r

    # thresh = 0.1
    # F_yf = F_yf_li if abs(alpha_f) < thresh else F_yf_nl
    # F_yr = F_yr_li if abs(alpha_r) < thresh else F_yr_nl
   # force clamp: ±1e3 N körül sosem lépünk túl

    F_yf = np.clip(F_yf_nl, -1e3, 1e3)
    F_yr = np.clip(F_yr_nl, -1e3, 1e3)

    # 4) differenciálegyenletek
    psi_ddot = (LF * F_yf - LR * F_yr) / Iz
    v_x_dot  = (Fx + m * v_y * psi_dot) / m
    v_y_dot  = (F_yf + F_yr) / m - v_x * psi_dot
    x_dot    = v_x * np.cos(yaw) - v_y * np.sin(yaw)
    y_dot    = v_x * np.sin(yaw) + v_y * np.cos(yaw)

    return np.array([v_x_dot, v_y_dot, psi_ddot, x_dot, y_dot, psi_dot])

def rk4_step(state, delta, dt, v_real_prev, v_real):
    """RK4 integrátor egy lépésnyi állapotfrissítéssel."""
    k1 = dynamics(state,              delta, dt, v_real_prev, v_real)
    k2 = dynamics(state + 0.5*dt*k1,  delta, dt, v_real_prev, v_real)
    k3 = dynamics(state + 0.5*dt*k2,  delta, dt, v_real_prev, v_real)
    k4 = dynamics(state +     dt*k3,  delta, dt, v_real_prev, v_real)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def main():
    # 1) CSV-k
    df_drive = pd.read_csv(DRIVE_CSV)   # time, speed, steering_angle
    df_odom  = pd.read_csv(ODOM_CSV)

    # 2) yaw kiszámolása
    if 'yaw' not in df_odom.columns:
        qw, qx, qy, qz = (df_odom[c].astype(float) for c in ['qw','qx','qy','qz'])
        df_odom['yaw'] = np.arctan2(2*(qw*qz + qx*qy),
                                    1-2*(qy*qy + qz*qz))

    # 3) közös időrács
    t_max = min(df_drive['time'].max(), df_odom['time'].max())
    dt = 0.02
    time_grid = np.arange(0.0, t_max+1e-9, dt)

    # 4) reindex+interpolate
    df_d_i = (df_drive.drop_duplicates('time')
                      .set_index('time')
                      .reindex(time_grid, method='nearest')
                      .reset_index().rename(columns={'index':'time'}))
    df_o_i = (df_odom.drop_duplicates('time')
                      .set_index('time')
                      .reindex(time_grid)
                      .interpolate()
                      .ffill().bfill()
                      .reset_index().rename(columns={'index':'time'}))

    # 5) v_real kiszámolása
    times = df_o_i['time'].to_numpy()
    xs    = df_o_i['x'].to_numpy()
    ys    = df_o_i['y'].to_numpy()
    v_real = np.zeros_like(times)
    v_real[1:] = np.hypot(xs[1:]-xs[:-1], ys[1:]-ys[:-1]) / dt

    # 6) Modell‐integráció RK4-gyel
    sim_states = []
    state = np.array([v_real[0], 0.0, 0.0, xs[0], ys[0], df_o_i.loc[0,'yaw']])
    for k in range(1, len(time_grid)):
        delta_cmd   = df_d_i.loc[k,'steering_angle']
        v_prev      = v_real[k-1]
        v_now       = v_real[k]
        state       = rk4_step(state, delta_cmd, dt, v_prev, v_now)
        sim_states.append([time_grid[k], *state])

    df_sim = pd.DataFrame(sim_states, columns=[
        'time','v_x_sim','v_y_sim','psi_dot_sim','x_sim','y_sim','yaw_sim'
    ])

    # 7) Hiba számítása
    x_m   = xs[1:];           y_m   = ys[1:]
    x_s   = df_sim['x_sim'].to_numpy(); y_s = df_sim['y_sim'].to_numpy()
    e_pos = np.hypot(x_s - x_m, y_s - y_m)

    yaw_m = df_o_i['yaw'].to_numpy()[1:]
    yaw_s = df_sim['yaw_sim'].to_numpy()
    e_yaw = wrap_diff(yaw_s, yaw_m)

    rmse_pos = np.sqrt((e_pos**2).mean())
    max_pos  = np.max(np.abs(e_pos))
    rmse_yaw = np.sqrt((e_yaw**2).mean())
    max_yaw  = np.max(np.abs(e_yaw))

    print(f"Position RMSE: {rmse_pos:.3f} m, max: {max_pos:.3f} m")
    print(f"Yaw      RMSE: {rmse_yaw:.3f} rad, max: {max_yaw:.3f} rad")

    # 8) Ábrák
    plt.figure(); plt.plot(x_m, y_m, label='measured')
    plt.plot(x_s, y_s, '--', label='model')
    plt.axis('equal'); plt.legend(); plt.title('Trajectory')

    plt.figure(); plt.plot(time_grid[1:], e_pos)
    plt.title('Position Error over Time')
    plt.xlabel('Time [s]'); plt.ylabel('Error [m]')

    plt.figure(); plt.plot(time_grid[1:], e_yaw)
    plt.title('Yaw Error over Time')
    plt.xlabel('Time [s]'); plt.ylabel('Error [rad]')

    if not args.no_plot:
        plt.show()

if __name__ == '__main__':
    main()
