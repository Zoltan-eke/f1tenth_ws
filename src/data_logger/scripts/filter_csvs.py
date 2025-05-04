#!/usr/bin/env python3
"""
filter_csvs.py

Offline szkript a "simulated" könyvtárban található
sim_drive.csv, sim_odom.csv és sim_tf.csv fájlok
időintervallumának összehangolására.

Támogatja mind a relatív 'time' oszlopot, mind az abszolút
'sec','nanosec' feldolgozást.
"""

import os, csv

# --- PATHS: állítsd be a saját workspace-ed szerint ---
WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
SIM_DIR        = os.path.join(WORKSPACE_ROOT,
                              'src/data_logger/logged_file/simulated')

DRIVE_FILE = os.path.join(SIM_DIR, 'sim_drive.csv')
ODOM_FILE  = os.path.join(SIM_DIR, 'sim_odom.csv')
TF_FILE    = os.path.join(SIM_DIR, 'sim_tf.csv')

ODOM_OUT = os.path.join(SIM_DIR, 'sim_odom_filt.csv')
TF_OUT   = os.path.join(SIM_DIR, 'sim_tf_filt.csv')
# ----------------------------------------------

def load_and_compute_times(path):
    """
    Beolvassa a CSV-t. Ha van 'time' oszlop, használja azt.
    Ha nincs, de van 'sec','nanosec', akkor abból számolja:
      abs_times = sec + nanosec*1e-9
      rel_times = abs_times - abs_times[0]
    Visszaadja: (rows, rel_times)
    """
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return [], []

    # Ha van time oszlop, közvetlenül float-oljuk
    if 'time' in rows[0]:
        times = [float(r['time']) for r in rows]
        return rows, times

    # különben abs sec,nanosec → relatív time
    if 'sec' in rows[0] and 'nanosec' in rows[0]:
        abs_times = [float(r['sec']) + float(r['nanosec'])*1e-9 for r in rows]
        t0 = abs_times[0]
        rel = [t - t0 for t in abs_times]
        # belerakjuk minden sorba a rel_time mezőt
        for r, t in zip(rows, rel):
            r['time'] = f"{t:.6f}"
        return rows, rel

    # nincs megfelelő oszlop
    raise RuntimeError(f"{path}: nincs 'time' vagy 'sec','nanosec' oszlop")

def filter_rows(rows, times, t_min, t_max):
    """Kiszűri a sorokat, ahol t_min ≤ times[i] ≤ t_max."""
    out = []
    for r, t in zip(rows, times):
        if t_min <= t <= t_max:
            out.append(r)
    return out

def write_csv(path, rows):
    """Kiírja a list of dicts-et, a fieldnames a keys() alapján."""
    if not rows:
        print(f"Üres a kimenet: {path}")
        return
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Írva: {path} ({len(rows)} sor)")

def main():
    # 1) sim_drive.csv → t_min, t_max
    drive_rows, drive_times = load_and_compute_times(DRIVE_FILE)
    if not drive_rows:
        print(f"Hiba: sim_drive.csv üres vagy hiányzik: {DRIVE_FILE}")
        return
    t_min, t_max = min(drive_times), max(drive_times)
    print(f"Drive időablak: {t_min:.3f} … {t_max:.3f} s")

    # 2) sim_odom szűrése
    odom_rows, odom_times = load_and_compute_times(ODOM_FILE)
    odom_filt = filter_rows(odom_rows, odom_times, t_min, t_max)
    write_csv(ODOM_OUT, odom_filt)

    # 3) sim_tf szűrése
    tf_rows, tf_times = load_and_compute_times(TF_FILE)
    tf_filt = filter_rows(tf_rows, tf_times, t_min, t_max)
    write_csv(TF_OUT, tf_filt)

if __name__ == '__main__':
    main()
