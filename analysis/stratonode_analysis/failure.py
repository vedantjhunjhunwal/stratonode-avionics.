"""Design-against-failure analyses.

* Brown-out: cold-cell voltage sag under worst-case load steps.
* Hold-up: capacitance to ride through a connector micro-interruption, with and without
  firmware load shedding, and with cold capacitor ESR.
* Solder-joint thermal-cycling fatigue (Engelmaier model, IPC-SM-785 / IPC-9701 form).
* Random-vibration board fatigue (Steinberg's allowable-deflection method).
"""
import math

from . import design as D
from . import power


# ---------------------------------------------------------------- brown-out
def pack_sag(cell_temp_C: float, soc: float, i_pack_A: float, n_parallel: int) -> dict:
    r_cell = power.cell_resistance(cell_temp_C)
    r_pack = r_cell * D.SERIES / n_parallel
    v_ocv = power.ocv(soc) * D.SERIES
    v_term = v_ocv - i_pack_A * r_pack
    return dict(r_cell_mohm=1000 * r_cell, r_pack_mohm=1000 * r_pack, v_ocv=v_ocv, v_terminal=v_term,
                margin_to_bms_V=v_term - D.BMS_CUTOFF_V, margin_to_buck_V=v_term - D.V_BUCK_MIN)


def worst_step_current(v_bat: float, heater_W_total: float) -> float:
    """All loads at peak (radio TX, payload inrush, sensors converting) plus both heaters on."""
    peak = power.board_input_power("cruise", v_bat, peak=True)["total"]
    return (peak + heater_W_total) / v_bat


# ---------------------------------------------------------------- hold-up
def holdup_capacitance_F(p_full_W: float, p_essential_W: float, shed: bool,
                         v1: float = D.V_HOLD_START, v2: float = D.V_BUCK_MIN,
                         t_drop: float = D.DROPOUT_S, t_shed: float = D.SHED_LATENCY_S) -> float:
    """Energy drawn from the bulk capacitor during a dropout, then C = 2E / (v1^2 - v2^2).
    The p_* values are battery-side powers (regulator losses already included)."""
    if shed:
        e = p_full_W * t_shed + p_essential_W * (t_drop - t_shed)
    else:
        e = p_full_W * t_drop
    return 2 * e / (v1 ** 2 - v2 ** 2)


# Capacitor behaviour at -55 C relative to 25 C (typical vendor curves, ASSUMED):
CAP_COLD = {
    "aluminium electrolytic": dict(c_ratio=0.70, esr_ratio=20.0, esr25_ohm=0.12),
    "aluminium polymer":      dict(c_ratio=0.95, esr_ratio=1.8, esr25_ohm=0.020),
}


def esr_step_V(kind: str, i_A: float, n_parallel: int) -> float:
    d = CAP_COLD[kind]
    return i_A * d["esr25_ohm"] * d["esr_ratio"] / n_parallel


# ---------------------------------------------------------------- solder fatigue (Engelmaier)
EPS_F = 0.325   # fatigue ductility coefficient, eutectic / near-eutectic solder


def engelmaier_c(t_sj_mean_C: float, cycles_per_day: float) -> float:
    return -0.442 - 6e-4 * t_sj_mean_C + 1.74e-2 * math.log(1 + cycles_per_day)


def engelmaier_cycles(delta_D: float, t_sj_mean_C: float, cycles_per_day: float = 1.0) -> float:
    """Median (50% failure) cycles to failure."""
    c = engelmaier_c(t_sj_mean_C, cycles_per_day)
    return 0.5 * (delta_D / (2 * EPS_F)) ** (1.0 / c)


def leadless_shear_strain(L_D_mm: float, d_alpha_ppm: float, dT: float, h_mm: float, F: float = 1.0) -> float:
    return F * (L_D_mm * d_alpha_ppm * 1e-6 * dT) / h_mm


def leaded_shear_strain(L_D_mm: float, d_alpha_ppm: float, dT: float, h_mm: float,
                        K_D_lb_in: float, area_mm2: float, F: float = 1.0) -> float:
    """Engelmaier compliant-lead form (inch-pound units inside, as published)."""
    L_in, h_in, A_in2 = L_D_mm / 25.4, h_mm / 25.4, area_mm2 / 645.16
    return F * K_D_lb_in * (L_in * d_alpha_ppm * 1e-6 * dT) ** 2 / (200.0 * A_in2 * h_in)


def weibull_scale(n50: float, fail_frac: float, beta: float = 4.0) -> float:
    """Cycles to reach fail_frac failures given the median, Weibull slope beta (IPC-9701 typical)."""
    return n50 * (math.log(1 - fail_frac) / math.log(0.5)) ** (1.0 / beta)


# package: (type, L_D mm = half the length between outer terminations, CTE mismatch ppm/K, joint height mm, extra)
PACKAGES = {
    "0603 resistor/MLCC": ("leadless", 0.80, 10.5, 0.08, {}),
    "0805 resistor/MLCC": ("leadless", 1.00, 10.5, 0.08, {}),
    "1206 current shunt": ("leadless", 1.60, 10.5, 0.10, {}),
    "1210 MLCC": ("leadless", 1.60, 10.5, 0.10, {}),
    "2512 current shunt": ("leadless", 3.18, 10.5, 0.10, {}),
    "LQFP-48 MCU": ("leaded", 6.36, 3.0, 0.05, dict(K_D=50.0, area=0.45)),
    "SOT-23 FET": ("leaded", 1.45, 5.0, 0.05, dict(K_D=400.0, area=0.30)),
}


def solder_life(package: str, t_hot_C: float, t_cold_C: float, cycles_per_day: float = 1.0) -> dict:
    kind, L_D, da, h, x = PACKAGES[package]
    dT = t_hot_C - t_cold_C
    if kind == "leadless":
        dd = leadless_shear_strain(L_D, da, dT, h)
    else:
        dd = leaded_shear_strain(L_D, da, dT, h, x["K_D"], x["area"])
    n50 = engelmaier_cycles(dd, 0.5 * (t_hot_C + t_cold_C), cycles_per_day)
    return dict(package=package, delta_T=dT, strain=dd, n50=n50, n_0p1pct=weibull_scale(n50, 0.001))


# ---------------------------------------------------------------- vibration (Steinberg)
def plate_fn_Hz(pcb: dict = D.PCB) -> float:
    """Simply-supported rectangular plate, fundamental mode (6-point mount approximated as SS edges)."""
    a, b, h = pcb["len_mm"] / 1000, pcb["wid_mm"] / 1000, pcb["thick_mm"] / 1000
    Dflex = pcb["E_GPa"] * 1e9 * h ** 3 / (12 * (1 - pcb["nu"] ** 2))
    m_s = pcb["mass_g"] / 1000 / (a * b)
    return (math.pi / 2) * math.sqrt(Dflex / m_s) * (1 / a ** 2 + 1 / b ** 2)


def steinberg(component_len_mm: float, c_const: float, r_pos: float = 1.0, psd: float = D.VIB_PSD_G2_HZ,
              pcb: dict = D.PCB) -> dict:
    fn = plate_fn_Hz(pcb)
    q = math.sqrt(fn)
    g_rms = math.sqrt(math.pi / 2 * fn * q * psd)
    z3 = 3 * 9.80665 * g_rms / (2 * math.pi * fn) ** 2 * 1000            # mm, 3-sigma
    B_in, h_in, L_in = pcb["len_mm"] / 25.4, pcb["thick_mm"] / 25.4, component_len_mm / 25.4
    z_allow = 0.00022 * B_in / (c_const * h_in * r_pos * math.sqrt(L_in)) * 25.4   # mm, 20e6 cycles
    return dict(fn_Hz=fn, Q=q, g_rms_response=g_rms, z3sigma_mm=z3, z_allow_mm=z_allow, margin=z_allow / z3)
