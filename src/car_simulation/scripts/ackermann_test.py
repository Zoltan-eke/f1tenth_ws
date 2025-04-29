#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# ---- PARAMÉTEREK ----
L = 0.32           # wheel_base [m]
v = 1.0            # sebesség [m/s]
dts = 0.01         # időlépés [s]
T = 10.0           # összes idő [s]
deltas = [0.0,     # egyenes
          0.1,     # enyhe kanyar
          0.3,     # közepes kanyar
          0.6]     # éles kanyar

# ---- INTEGRÁCIÓ ----
t = np.arange(0, T, dts)
plt.figure(figsize=(6,6))
for delta in deltas:
    x = 0.0; y = 0.0; theta = 0.0
    xs = []; ys = []
    for _ in t:
        xs.append(x); ys.append(y)
        # kinematika:
        x   += v * np.cos(theta) * dts
        y   += v * np.sin(theta) * dts
        theta += v / L * np.tan(delta) * dts
    plt.plot(xs, ys, label=f"δ={delta:.2f} rad")

# ---- ÁBRA BEÁLLÍTÁSOK ----
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title("Ackermann kinematic test")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.tight_layout()
plt.show()
