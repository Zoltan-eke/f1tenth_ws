#!/usr/bin/env python3
import subprocess, yaml, tempfile, os, numpy as np

# 1) B és C tartomány
B_vals = np.linspace(5.0, 20.0, 5)
C_vals = np.linspace(1.5, 2.5, 5)

# 2) Fix C_lin, D értékek
C_lin = 150.0
D     = 10.0

best = {'score': 1e9, 'B': None, 'C': None}

# 3) YAML útvonala
YAML_PATH = os.path.expanduser('~/f1tenth_ws/src/car_description/config/params.yaml')

for B in B_vals:
    for C in C_vals:
        # 4) Paraméterek frissítése,
        # kiolvassuk, módosítjuk, és ideiglenesen átírjuk
        with open(YAML_PATH) as f:
            cfg = yaml.safe_load(f)
        rp = cfg.setdefault('ros__parameters', {})
        rp.update({'B': float(B), 'C': float(C),
                   'C_lin': float(C_lin), 'D': float(D)})

        # ideiglenesen írjunk ki egy temp YAML-t
        tmp = tempfile.NamedTemporaryFile('w', delete=False, suffix='.yaml')
        yaml.safe_dump(cfg, tmp)
        tmp.flush(); tmp.close()

        # 5) lefuttatjuk az offline_validation-t
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

# 6) Legjobb megtalálása és params.yaml frissítése
print("=== BEST ===", best)
with open(YAML_PATH) as f:
    cfg = yaml.safe_load(f)
rp = cfg.setdefault('ros__parameters', {})
rp['B'] = float(best['B'])
rp['C'] = float(best['C'])
with open(YAML_PATH, 'w') as f:
    yaml.safe_dump(cfg, f)

print(f"Frissítve a {YAML_PATH}: B={best['B']}, C={best['C']}")
