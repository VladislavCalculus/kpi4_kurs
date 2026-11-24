"""Optional export: python3 -m pip install matplotlib; python3 plot.py."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
with (root / "route.csv").open() as stream:
    records = list(csv.DictReader(stream))
with (root / "waypoints.csv").open() as stream:
    plan = list(csv.DictReader(stream))
fig, axes = plt.subplots(1, 2, figsize=(10, 5), constrained_layout=True)
axes[0].plot([float(r["east_m"]) for r in plan], [float(r["north_m"]) for r in plan],
             "o--", color="#555555", label="Planned route")
axes[0].plot([float(r["east_m"]) for r in records], [float(r["north_m"]) for r in records],
             color="#147b6b", linewidth=2, label="True trajectory")
bad = [r for r in records if r["state_reason"] != "accepted"]
axes[0].plot([float(r["east_m"]) for r in bad], [float(r["north_m"]) for r in bad],
             color="#c24e28", linewidth=3, label="No new command")
for row in plan[1:]:
    axes[0].add_patch(plt.Circle((float(row["east_m"]), float(row["north_m"])),
                                 3, color="#555555", fill=False, linestyle=":"))
axes[0].set(xlabel="East, m", ylabel="North, m", title="Three segments; noise +/-0.35 m")
axes[0].set_aspect("equal", adjustable="box")
axes[1].plot([float(r["time_s"]) for r in records],
             [abs(float(r["true_cross_track_m"])) for r in records], color="#147b6b")
axes[1].axvspan(4, 6, color="#c24e28", alpha=0.15, label="Stale GPS [4,6) s")
axes[1].axhline(8, color="#555555", linestyle=":", label="8 m limit")
axes[1].set(xlabel="Simulation time, s", ylabel="Absolute cross-track error, m")
for ax in axes:
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
fig.savefig(root / "trajectory.png", dpi=150)
