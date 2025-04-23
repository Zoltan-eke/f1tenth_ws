#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# CSV elérési út
csv_path = '/home/ezo/f1tenth_ws/src/data_logger/logged_file/filtered/filtered_odom.csv'

# Beolvasás
odom_df = pd.read_csv(csv_path)

# Időbélyeg összevonása másodpercbe
odom_df['time'] = odom_df['sec'] + odom_df['nanosec'] * 1e-9

# Relatív idő kiszámítása
odom_df['rel_time'] = odom_df['time'] - odom_df['time'].min()

# Szűrés: csak a megadott időintervallum között
t_min = 0.00   # másodperc 48
t_max = 180.0  # másodperc 132
filtered_df = odom_df[(odom_df['rel_time'] > t_min) & (odom_df['rel_time'] < t_max)]


# Típuskonverzió és NaN kiszűrése
x_vals = pd.to_numeric(filtered_df['x'], errors='coerce')
y_vals = pd.to_numeric(filtered_df['y'], errors='coerce')

# Csak érvényes (nem NaN) értékek
valid = x_vals.notna() & y_vals.notna()
x_vals = x_vals[valid].to_numpy()
y_vals = y_vals[valid].to_numpy()

# Plotolás
plt.figure()
plt.plot(filtered_df['x'].to_numpy(), filtered_df['y'].to_numpy(), marker='.', linestyle='-')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title(f'Odom pálya ({t_min}s – {t_max}s között)')
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()
