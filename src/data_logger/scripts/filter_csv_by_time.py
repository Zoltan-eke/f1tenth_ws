#!/usr/bin/env python3
import os
import pandas as pd

# --- Beállítások ---
input_dir = "/home/ezo/f1tenth_ws/src/data_logger/logged_file/raw"
output_dir = "/home/ezo/f1tenth_ws/src/data_logger/logged_file/filtered"
os.makedirs(output_dir, exist_ok=True)

# Szűrés időtartomány (relatív másodpercben)
T_MIN = 48.0
T_MAX = 132.0

# Fájlok, amiket feldolgozunk
files = {
    "odom": "odom.csv",
    "imu": "imu.csv",
    "joint_states": "joint_states.csv",
    "ackermann_cmd": "ackermann_cmd.csv"
}

for name, filename in files.items():
    input_path = os.path.join(input_dir, filename)
    output_path = os.path.join(output_dir, f"filtered_{filename}")
    
    if not os.path.exists(input_path):
        print(f"[!] Nincs ilyen fájl: {input_path}")
        continue

    # CSV betöltése és időbélyeg kiszámítása
    df = pd.read_csv(input_path)
    df["time"] = df["sec"] + df["nanosec"] * 1e-9
    df["rel_time"] = df["time"] - df["time"].min()

    # Szűrés
    df_filtered = df[(df["rel_time"] >= T_MIN) & (df["rel_time"] <= T_MAX)]

    # Mentés
    df_filtered.to_csv(output_path, index=False)
    print(f"[✓] Mentve: {output_path} ({len(df_filtered)} sor)")
