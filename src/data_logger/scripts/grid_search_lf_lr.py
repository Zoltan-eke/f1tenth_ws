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
import copy

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
NS = next(iter(master_cfg))

best = {'L_total':None, 'LF_frac':None, 'pos':None, 'yaw':None, 'score':np.inf}

# ideiglenes fájl, amibe mindig az aktualizált master_cfg-t írjuk
temp_yaml = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)
temp_yaml.close()



for L_total in L_vals:
    for frac in LF_fracs:
        LF = float(frac * L_total)
        LR = float(L_total - LF)

        # mély másolat, hogy a beágyazott ros__parameters dict ne legyen
        # megosztott referencia iterációk között, csak a geom mezőket írjuk felül
        cfg = copy.deepcopy(master_cfg)
        rp = cfg[NS]['ros__parameters']
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

        # pozíció és yaw RMSE kinyerése a kimenetből
        pos_rmse = yaw_rmse = None
        for line in output.splitlines():
            if line.startswith('Position RMSE:'):
                pos_rmse = float(line.split()[2])
            if line.startswith('Yaw      RMSE:'):
                # formátum: Yaw      RMSE: X.XXXX rad, max: Y.YYYY rad
                yaw_rmse = float(line.split()[2])

        if pos_rmse is None or yaw_rmse is None:
            print(f"[L={L_total:.3f}, frac={frac:.2f}] Nem található RMSE a kimenetben.")
            continue

        score = 0.5*pos_rmse + 0.5*yaw_rmse
        print(f"[L={L_total:.3f}, LF_frac={frac:.2f}] pos={pos_rmse:.3f}, "
              f"yaw={yaw_rmse:.4f}, score={score:.4f}")

        # legjobb frissítése
        if score < best['score']:
            best.update(L_total=L_total, LF_frac=frac, pos=pos_rmse,
                         yaw=yaw_rmse, score=score)

print("\n=== Legjobb eredmény ===")
print(f"L_total = {best['L_total']:.3f} m, "
      f"LF = {best['LF_frac']*100:.1f}%·L_total → "
      f"pos_rmse = {best['pos']:.3f}, yaw_rmse = {best['yaw']:.4f}, "
      f"score = {best['score']:.4f}")

# --- szél-ellenőrzés: az optimum a keresési tartomány szélén van-e ---
if np.isclose(best['L_total'], L_vals.min()) or np.isclose(best['L_total'], L_vals.max()):
    print(f"[WARNING] L_total optimuma a keresési tartomány szélén van (L_total={best['L_total']:.3f}, "
          f"tartomány=[{L_vals.min():.3f}, {L_vals.max():.3f}]) — érdemes szélesíteni a rácsot.")
if np.isclose(best['LF_frac'], LF_fracs.min()) or np.isclose(best['LF_frac'], LF_fracs.max()):
    print(f"[WARNING] LF_frac optimuma a keresési tartomány szélén van (LF_frac={best['LF_frac']:.2f}, "
          f"tartomány=[{LF_fracs.min():.2f}, {LF_fracs.max():.2f}]) — érdemes szélesíteni a rácsot.")

# Natív float-okkal mentünk YAML-be
LF_val = float(best['LF_frac'] * best['L_total'])
LR_val = float(best['L_total'] - LF_val)
with open('best_l_tot_lf.yaml', 'w') as f:
    yaml.dump({
        'LF': LF_val,
        'LR': LR_val,
        'score': float(best['score']),
    }, f, default_flow_style=False)

#
# Végén frissítjük a MASTER_PARAMS fájlt a talált optimum keresztül:
#
rp = master_cfg[NS]['ros__parameters']
rp['wheelbase_front'] = LF_val
rp['wheelbase_rear']  = LR_val
with open(MASTER_PARAMS, 'w') as f:
    yaml.dump(master_cfg, f, default_flow_style=False)
print(f"Frissítve: {MASTER_PARAMS}")
