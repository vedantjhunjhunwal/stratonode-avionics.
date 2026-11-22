"""Thermal models: insulated enclosures in thin, cold air, and component junction temperatures.

Enclosure: inner air/contents at T_in, a foam wall (conduction), then the outer
surface loses heat to the vented fuselage air by natural convection and to the
fuselage walls by radiation. Natural-convection film coefficient scales with
sqrt(p/p0) (laminar Nu ~ Gr^1/4 and Gr ~ rho^2), which is why the same box needs
less heat per kelvin at altitude than on the ground, while the colder air demands
more kelvin. Radiation does not care about pressure and dominates up high.
"""
import math

from . import design as D

SIGMA = 5.670374e-8
P0 = 101325.0


def box_area_m2(box: D.Box, at_outer: bool = False) -> float:
    l, w, h = (x / 1000.0 for x in box.inner_mm)
    if at_outer:
        t = box.insulation_mm / 1000.0
        l, w, h = l + 2 * t, w + 2 * t, h + 2 * t
    return 2 * (l * w + l * h + w * h)


def h_natural(p_Pa: float) -> float:
    return D.H_NAT_SL * math.sqrt(p_Pa / P0)


def heat_loss_W(box: D.Box, t_in_C: float, t_amb_C: float, p_Pa: float) -> float:
    """Steady heat flow out of the box (W), solving for the outer-surface temperature."""
    a_in, a_out = box_area_m2(box), box_area_m2(box, True)
    a_mean = math.sqrt(a_in * a_out)
    g_wall = box.k_ins * a_mean / (box.insulation_mm / 1000.0)    # W/K
    t_s = 0.5 * (t_in_C + t_amb_C)
    for _ in range(100):
        tm = 0.5 * (t_s + t_amb_C) + 273.15
        h_rad = 4 * box.emissivity * SIGMA * tm ** 3
        g_out = (h_natural(p_Pa) + h_rad) * a_out
        q = (t_in_C - t_amb_C) / (1 / g_wall + 1 / g_out)
        t_new = t_amb_C + q / g_out
        if abs(t_new - t_s) < 1e-9:
            break
        t_s = t_new
    return q


def conductance_W_K(box: D.Box, t_amb_C: float, p_Pa: float) -> float:
    return heat_loss_W(box, box.target_C, t_amb_C, p_Pa) / (box.target_C - t_amb_C)


def heater_required_W(box: D.Box, t_amb_C: float, p_Pa: float, internal_W: float = None) -> float:
    """Heater power to hold the box at its target temperature (0 if internal heat suffices)."""
    internal = box.internal_W if internal_W is None else internal_W
    if t_amb_C >= box.target_C:
        return 0.0
    return max(0.0, heat_loss_W(box, box.target_C, t_amb_C, p_Pa) - internal)


def unheated_temp_C(box: D.Box, t_amb_C: float, p_Pa: float, internal_W: float) -> float:
    """Steady internal temperature with only internal dissipation (bisection)."""
    lo, hi = t_amb_C, t_amb_C + 300
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if heat_loss_W(box, mid, t_amb_C, p_Pa) > internal_W:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def warmup_minutes(box: D.Box, heater_W: float, t_start_C: float, t_amb_C: float, p_Pa: float,
                   heat_capacity_J_K: float) -> float:
    """Time for the heater to raise the box contents from t_start to target (explicit Euler)."""
    t, dt, s = t_start_C, 1.0, 0.0
    while t < box.target_C:
        q = heater_W + box.internal_W - heat_loss_W(box, t, t_amb_C, p_Pa)
        if q <= 0:
            return math.inf
        t += q * dt / heat_capacity_J_K
        s += dt
        if s > 48 * 3600:
            return math.inf
    return s / 60.0


# ---- junction temperatures ---------------------------------------------------------------------
# theta_JA from datasheets (JEDEC high-K board, still air at sea level), split into the
# junction-to-board part (fixed) and board-to-air part (scales with the film coefficient).
PARTS = {
    "TPS62933 buck (SOT-583)": dict(theta_ja=74.2, theta_jb=26.5, t_min=-40, t_max=150),
    "AP2112K LDO (SOT-23-5)": dict(theta_ja=180.0, theta_jb=55.0, t_min=-40, t_max=125),
    "STM32G474 MCU (LQFP-48)": dict(theta_ja=55.0, theta_jb=35.0, t_min=-40, t_max=125),
    "heater FET AO3400A (SOT-23)": dict(theta_ja=125.0, theta_jb=60.0, t_min=-55, t_max=150),
}


def theta_ja_at(part: str, p_Pa: float, t_air_C: float) -> float:
    d = PARTS[part]
    tm = t_air_C + 273.15
    h_sl = D.H_NAT_SL * 2.0 + 4 * 0.9 * SIGMA * 298.15 ** 3     # PCB-scale convection + radiation, sea level 25 C
    h_alt = D.H_NAT_SL * 2.0 * math.sqrt(p_Pa / P0) + 4 * 0.9 * SIGMA * tm ** 3
    return d["theta_jb"] + (d["theta_ja"] - d["theta_jb"]) * h_sl / h_alt


def junction_C(part: str, p_W: float, t_air_C: float, p_Pa: float) -> float:
    return t_air_C + p_W * theta_ja_at(part, p_Pa, t_air_C)
