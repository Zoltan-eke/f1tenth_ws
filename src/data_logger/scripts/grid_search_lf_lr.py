#!/usr/bin/env python3
"""
grid_search_l_tot_lf.py

Grid‐search a wheelbase (L_total) és az LF/LR arány optimalizálásához,
egyetlen master params.yaml alapján.

Használat:
  cd ~/f1tenth_ws/src/data_logger/scripts
  chmod +x grid_search_l_tot_lf.py
  ./grid_search_l_tot_lf.py
"""

import numpy as np
import yaml
import subprocess
import tempfile
import os

# --- TESTRESZABÁS: offline_validation.py elérési útja ---
VALIDATION_SCRIPT = os.path.expanduser(
    '~/f1tenth_ws/src/data_logger/scripts/offline_validation.py'
)

# --- Paraméterrács beállítása ---
# wheelbase nominal és arány rácsok
L_nom    = 0.32
L_vals   = np.linspace(L_nom - 0.005, L_nom + 0.005, 5)
LF_fracs = np.linspace(0.2, 0.8, 13)

# az egyetlen master YAML, amiben minden modell‑ és gumi‑paraméter is benne van
MASTER_PARAMS = os.path.expanduser(
    '~/f1tenth_ws/src/car_description/config/params.yaml'
)

# beolvassuk egyszer a teljes master configot
with open(MASTER_PARAMS, 'r') as f:
    master_cfg = yaml.safe_load(f)

best = {'L_total':None, 'LF_frac':None, 'yaw_rmse':np.inf}

# ideiglenes fájl, amibe mindig az aktualizált master_cfg-t írjuk
temp_yaml = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)
temp_yaml.close()



for L_total in L_vals:
    for frac in LF_fracs:
        LF = float(frac * L_total)
        LR = float(L_total - LF)

        # a teljes master_cfg másolata, itt csak a geom mezőket írjuk felül
        cfg = dict(master_cfg)
        rp = cfg.setdefault('ros__parameters', {})
        rp['wheelbase_front'] = LF
        rp['wheelbase_rear']  = LR

        # ideiglenesen kiírjuk a temp fájlba
        with open(temp_yaml.name, 'w') as f:
            yaml.dump(cfg, f, default_flow_style=False)

        # offline_validation.py futtatása --no-plot opcióval a temp param-mal
        try:
            output = subprocess.check_output([
                'python3', VALIDATION_SCRIPT,
                '--params', temp_yaml.name,
                '--no-plot'
            ], stderr=subprocess.STDOUT, text=True)
        except subprocess.CalledProcessError as e:
            print(f"[L={L_total:.3f}, frac={frac:.2f}] Hibával kilépett:")
            print(e.output)
            continue

        # yaw RMSE kinyerése a kimenetből
        yaw_rmse = None
        for line in output.splitlines():
            if line.startswith('Yaw      RMSE:'):
                # formátum: Yaw      RMSE: X.XXXX rad, max: Y.YYYY rad
                parts = line.split()
                yaw_rmse = float(parts[2])
                break

        if yaw_rmse is None:
            print(f"[L={L_total:.3f}, frac={frac:.2f}] Nem található yaw RMSE a kimenetben.")
            continue
        print(f"[L={L_total:.3f}, LF_frac={frac:.2f}] yaw_rmse = {yaw_rmse:.4f}")

        # legjobb frissítése
        if yaw_rmse < best['yaw_rmse']:
            best.update(L_total=L_total, LF_frac=frac, yaw_rmse=yaw_rmse)

print("\n=== Legjobb eredmény ===")
print(f"L_total = {best['L_total']:.3f} m, "
      f"LF = {best['LF_frac']*100:.1f}%·L_total → "
      f"yaw_rmse = {best['yaw_rmse']:.4f}")

# Natív float-okkal mentünk YAML-be
LF_val = float(best['LF_frac'] * best['L_total'])
LR_val = float(best['L_total'] - LF_val)
with open('best_l_tot_lf.yaml', 'w') as f:
    yaml.dump({
        'LF': LF_val,
        'LR': LR_val
    }, f, default_flow_style=False)

#
# Végén frissítjük a MASTER_PARAMS fájlt a talált optimum keresztül:
#
rp = master_cfg.setdefault('ros__parameters', {})
rp['wheelbase_front'] = float(best['LF_frac'] * best['L_total'])
rp['wheelbase_rear']  = float(best['L_total'] - best['LF_frac'] * best['L_total'])
with open(MASTER_PARAMS, 'w') as f:
    yaml.dump(master_cfg, f, default_flow_style=False)
print(f"Frissítve: {MASTER_PARAMS}")
