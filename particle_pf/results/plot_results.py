#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

res_dir = Path(__file__).parent
errors_csv = res_dir / "errors.csv"
trace_csv  = res_dir / "trace.csv"

err = pd.read_csv(errors_csv)
print("\nHata İstatistikleri:")
print(err[["trans_err", "rot_err"]].describe().to_string(float_format="%.4f"))

tr  = pd.read_csv(trace_csv)

x_gt  = tr["x_gt" ].astype(float).to_numpy()
y_gt  = tr["y_gt" ].astype(float).to_numpy()
x_est = tr["x_est"].astype(float).to_numpy()
y_est = tr["y_est"].astype(float).to_numpy()

plt.figure(figsize=(6, 6))
plt.plot(x_gt,  y_gt,  label="Ground-Truth", linewidth=2)
plt.plot(x_est, y_est, label="PF Estimate", linewidth=2)
plt.axis("equal")
plt.legend()
plt.title("Particle Filter vs. Ground-Truth Path")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.grid(True)

out_png = res_dir / "path_compare.png"
plt.savefig(out_png, dpi=300, bbox_inches="tight")
print(f"\nGrafik kaydedildi - {out_png}")
