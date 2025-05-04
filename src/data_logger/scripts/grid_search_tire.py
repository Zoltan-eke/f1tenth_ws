#!/usr/bin/env python3
import numpy as np
import yaml, subprocess, tempfile, os, sys

# --- Fájlok elérési útja ---
MASTER = os.path.expanduser(
    '~/f1tenth_ws/src/car_description/config/params.yaml'
)
VALID  = os.path.expanduser(
    '~/f1tenth_ws/src/data_logger/scripts/offline_validation.py'
)

C_lin_vals = np.linspace(50,300,11)
D_vals     = np.linspace(10,200,11)

best = {'C_lin':None,'D':None,'score':np.inf}
tmp  = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)
tmp.close()

# Eredeti master YAML beolvasása
with open(MASTER, 'r') as f:
    base_cfg = yaml.safe_load(f)

print(f"[grid_search_tire] Master params: {MASTER}")

for C_lin in C_lin_vals:
    for D in D_vals:
        # Másoljuk a teljes konfigurációt, és írjuk felül a gumiparamétereket
        cfg = dict(base_cfg)
        rp  = cfg.setdefault('ros__parameters', {})
        rp['C_lin'] = float(C_lin)
        rp['D']     = float(D)
        with open(tmp.name,'w') as f:
            yaml.dump(cfg, f, default_flow_style=False)

        # Offline validáció futtatása
        try:
            out = subprocess.check_output([
                sys.executable, VALID, '--params', tmp.name, '--no-plot'
            ], stderr=subprocess.STDOUT, text=True)
        except subprocess.CalledProcessError as e:
            print(f"[C_lin={C_lin}, D={D}] Hiba:\n{e.output}")
            continue

        # RMSE kinyerése
        pos = yaw = None
        for line in out.splitlines():
            if line.startswith('Position RMSE:'):
                pos = float(line.split()[2])
            if line.startswith('Yaw      RMSE:'):
                yaw = float(line.split()[2])
        if pos is None or yaw is None:
            print(f"[C_lin={C_lin}, D={D}] Nincs RMSE az outputban.")
            continue

        score = 0.5*pos + 0.5*yaw
        print(f"[C_lin={C_lin:.1f}, D={D:.1f}] pos={pos:.3f}, yaw={yaw:.3f}, score={score:.3f}")
        if score < best['score']:
            best.update(C_lin=C_lin, D=D, score=score)

# Eredmény kiírása
print("\n=== BEST ===")
print(f"C_lin = {best['C_lin']:.1f}, D = {best['D']:.1f}, score = {best['score']:.3f}")

# Master YAML frissítése
rp = base_cfg.setdefault('ros__parameters', {})
rp['C_lin'] = float(best['C_lin'])
rp['D']     = float(best['D'])
with open(MASTER, 'w') as f:
    yaml.dump(base_cfg, f, default_flow_style=False)

print(f"[grid_search_tire] Frissítve a master params.yaml: C_lin={best['C_lin']}, D={best['D']}")
