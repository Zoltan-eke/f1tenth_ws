#!/usr/bin/env python3
import subprocess, yaml, tempfile, os, numpy as np

# 1) B és C tartomány
B_vals = np.linspace(5.0, 20.0, 5)
C_vals = np.linspace(1.5, 2.5, 5)

best = {'score': 1e9, 'B': None, 'C': None}

# 2) YAML útvonala
YAML_PATH = os.path.expanduser('~/f1tenth_ws/src/car_description/config/params.yaml')

for B in B_vals:
    for C in C_vals:
        # 3) Paraméterek frissítése: a master YAML-t (beágyazott
        # <namespace>: -> ros__parameters: struktúra) frissen beolvassuk,
        # csak B/C-t írjuk felül, C_lin/D a masterben aktuálisan
        # kalibrált értéken marad
        with open(YAML_PATH) as f:
            cfg = yaml.safe_load(f)
        ns = next(iter(cfg))
        rp = cfg[ns]['ros__parameters']
        rp['B'] = float(B)
        rp['C'] = float(C)

        # ideiglenesen írjunk ki egy temp YAML-t
        tmp = tempfile.NamedTemporaryFile('w', delete=False, suffix='.yaml')
        yaml.safe_dump(cfg, tmp)
        tmp.flush(); tmp.close()

        # 4) lefuttatjuk az offline_validation-t
        cmd = [
            'python3', 'offline_validation.py',
            '--params', tmp.name, '--no-plot'
        ]
        out = subprocess.check_output(cmd, universal_newlines=True)
        # például: "Position RMSE: 0.275 m, ...\nYaw      RMSE: 1.053 rad"
        pos = float(out.split('Position RMSE:')[1].split('m')[0])
        yaw = float(out.split('Yaw      RMSE:')[1].split('rad')[0])
        score = 0.5*pos + 0.5*yaw

        # --- NAN-szűrés ---
        if np.isnan(score):
            print(f" → B={B:.2f}, C={C:.2f} → nan, kihagyva")
            os.unlink(tmp.name)
            continue

        print(f"[B={B:.2f}, C={C:.2f}] pos={pos:.3f}, yaw={yaw:.3f}, score={score:.3f}")

        if score < best['score']:
            best.update({'score': score, 'B': B, 'C': C})

        os.unlink(tmp.name)

# 5) Legjobb megtalálása és params.yaml frissítése
print("=== BEST ===", best)

# --- szél-ellenőrzés: az optimum a keresési tartomány szélén van-e ---
if np.isclose(best['B'], B_vals.min()) or np.isclose(best['B'], B_vals.max()):
    print(f"[WARNING] B optimuma a keresési tartomány szélén van (B={best['B']:.2f}, "
          f"tartomány=[{B_vals.min():.2f}, {B_vals.max():.2f}]) — érdemes szélesíteni a rácsot.")
if np.isclose(best['C'], C_vals.min()) or np.isclose(best['C'], C_vals.max()):
    print(f"[WARNING] C optimuma a keresési tartomány szélén van (C={best['C']:.2f}, "
          f"tartomány=[{C_vals.min():.2f}, {C_vals.max():.2f}]) — érdemes szélesíteni a rácsot.")

with open(YAML_PATH) as f:
    cfg = yaml.safe_load(f)
ns = next(iter(cfg))
rp = cfg[ns]['ros__parameters']
rp['B'] = float(best['B'])
rp['C'] = float(best['C'])
with open(YAML_PATH, 'w') as f:
    yaml.safe_dump(cfg, f)

print(f"Frissítve a {YAML_PATH}: B={best['B']}, C={best['C']}")
