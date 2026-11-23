"""Optional figure export: python3 -m pip install matplotlib; python3 plot.py."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 1, figsize=(9, 6), constrained_layout=True)
for mode, label, color in (("baseline", "Without correction", "#c24e28"),
                           ("corrected", "With correction", "#147b6b")):
    with (root / f"{mode}.csv").open() as stream:
        data = list(csv.DictReader(stream))
    east = [float(r["east_m"]) for r in data]
    north = [float(r["north_m"]) for r in data]
    seconds = [float(r["time_s"]) for r in data]
    cross = [float(r["cross_track_m"]) for r in data]
    axes[0].plot(east, north, label=label, color=color, linewidth=2)
    axes[0].plot(east[-1], north[-1], "o", color=color)
    axes[1].plot(seconds, cross, label=label, color=color, linewidth=2)
axes[0].plot([0, 100], [0, 0], "--", color="#555555", label="Planned segment")
axes[0].add_patch(plt.Circle((100, 0), 3, color="#147b6b", fill=False, linestyle=":"))
axes[0].set(xlabel="East, m", ylabel="North, m", title="100 m segment; lateral drift 2 m/s")
axes[0].set_aspect("equal", adjustable="datalim")
axes[0].legend(loc="upper left", fontsize=9)
axes[1].axhline(5, color="#555555", linestyle=":", label="5 m acceptance limit")
axes[1].set(xlabel="Simulation time, s", ylabel="Cross-track error, m")
axes[1].legend(loc="upper left", fontsize=9)
for ax in axes:
    ax.grid(alpha=0.25)
fig.savefig(root / "trajectory.svg")
svg = root / "trajectory.svg"
svg.write_text("\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n")
fig.savefig(root / "trajectory.png", dpi=150)
