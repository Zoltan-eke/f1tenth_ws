#!/usr/bin/env python3
import os, csv

# --- TESTRESZABÁS: igazítsd a workspace-edhez ---
WORKSPACE_ROOT = os.path.expanduser('~/f1tenth_ws')
FILTERED_DIR   = os.path.join(WORKSPACE_ROOT,
                              'src/data_logger/logged_file/filtered')
SIM_DIR        = os.path.join(WORKSPACE_ROOT,
                              'src/data_logger/logged_file/simulated')
IN_FILE        = os.path.join(FILTERED_DIR, 'drive.csv')
OUT_FILE       = os.path.join(SIM_DIR, 'sim_drive.csv')
MAX_GAP = 0.5  # másodperc
# ----------------------------------------------

def main():
    if not os.path.isfile(IN_FILE):
        print(f"Hiba: nem található a bemeneti fájl: {IN_FILE}")
        return

    with open(IN_FILE, 'r') as fin:
        reader = csv.DictReader(fin)
        rows = list(reader)
    if not rows:
        print("Hiba: üres a bemeneti CSV!")
        return

    # abszolút idők (s)
    abs_times = [float(r['sec']) + float(r['nanosec'])*1e-9 for r in rows]

    # előkészítés: kimeneti lista
    filtered = []
    # első sor mindig benne van
    prev_t = abs_times[0]
    filtered.append( (0, rows[0], prev_t) )

    # végig a többi soron
    for i in range(1, len(rows)):
        t = abs_times[i]
        dt = t - prev_t
        if 0 <= dt <= MAX_GAP:
            filtered.append((i, rows[i], t))
            prev_t = t
        # ha túl nagy ugrás, skip, prev_t nem változik

    # írás sim_drive.csv-be
    os.makedirs(SIM_DIR, exist_ok=True)
    with open(OUT_FILE, 'w', newline='') as fout:
        writer = csv.writer(fout)
        writer.writerow(['time', 'speed', 'steering_angle'])
        t0 = filtered[0][2]
        for idx, r, t in filtered:
            t_rel = t - t0
            writer.writerow([f"{t_rel:.6f}", r['speed'], r['steering_angle']])

    print(f"Kész: {OUT_FILE} ({len(filtered)} sor, {len(rows)-len(filtered)} kihagyva)")

if __name__ == '__main__':
    main()
