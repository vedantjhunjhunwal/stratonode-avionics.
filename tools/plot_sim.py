"""Plot one simulated flight: results/sim/seed1.csv -> results/flight_sim.png"""
import csv, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
rows = list(csv.DictReader(open(os.path.join(root, "results", "sim", sys.argv[1] if len(sys.argv) > 1 else "seed1.csv"))))
col = lambda k: [float(r[k]) for r in rows]
t = col("t_h")
fig, ax = plt.subplots(3, 1, figsize=(10, 8.5), sharex=True)
a2 = ax[0].twinx()
ax[0].plot(t, [a / 1000 for a in col("alt_m")], "k", lw=1.5, label="altitude")
a2.plot(t, col("t_amb_c"), "tab:blue", lw=1.2, label="outside air")
ax[0].set_ylabel("altitude (km)"); a2.set_ylabel("outside air (°C)", color="tab:blue")
ax[0].set_title("StratoNode closed-loop flight simulation (firmware in the loop, seed 1)")
ax[1].plot(t, col("t_batt_c"), "tab:red", label="battery box (set-point +5 °C)")
ax[1].plot(t, col("t_bay_c"), "tab:purple", label="avionics bay (set-point -25 °C)")
ax[1].plot(t, col("t_amb_c"), "tab:blue", lw=0.8, alpha=0.6, label="outside air")
ax[1].axhline(-40, color="gray", ls="--", lw=0.8); ax[1].text(0.1, -38.5, "-40 °C part rating", fontsize=8, color="gray")
ax[1].set_ylabel("°C"); ax[1].legend(fontsize=8, loc="lower left")
ax[2].plot(t, col("p_total_w"), "tab:green", lw=0.6, label="battery power (W)")
ax[2].plot(t, [d * 12.6 for d in col("duty_batt")], "tab:red", lw=0.8, label="battery heater (W, approx.)")
ax[2].set_ylabel("W"); ax[2].set_xlabel("mission time (h)"); ax[2].legend(fontsize=8, loc="upper left")
for a in ax: a.grid(alpha=0.3)
ax[1].axvspan(7.0, 7.333, color="orange", alpha=0.15)
ax[1].text(7.02, 8, "NTC open:\nfallback duty", fontsize=7)
fig.tight_layout()
fig.savefig(os.path.join(root, "results", "flight_sim.png"), dpi=110)
