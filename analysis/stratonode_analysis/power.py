"""Power budget and battery sizing for the StratoNode avionics node.

Per flight phase: sum the rail loads, push them back through the LDO and buck loss
models to the battery, add the heater power that the thermal model requires, and
integrate energy over the mission. Then size the 4S pack (number of cells in
parallel) for that energy with cold, ageing and reserve derating.
"""
from dataclasses import dataclass
import math

from . import design as D
from .atmosphere import isa
from . import thermal


def rail_power(phase: str, peak: bool = False) -> dict:
    """Power drawn from the 3V3 and 5V rails in a phase (W)."""
    out = {"3V3": 0.0, "5V": 0.0}
    for ld in D.LOADS:
        if phase in ld.phases:
            out[ld.rail] += ld.p_peak_W if peak else ld.p_on_W
    return out


def board_input_power(phase: str, v_bat: float, peak: bool = False) -> dict:
    """Push rail loads back to the battery terminals. Returns a breakdown in W."""
    r = rail_power(phase, peak)
    i_3v3 = r["3V3"] / D.V_3V3
    if i_3v3 > D.LDO_LIMIT_A:
        raise ValueError(f"3V3 rail {i_3v3:.2f} A exceeds LDO limit")
    ldo_in = r["3V3"] + D.ldo_loss_W(i_3v3)
    p5 = r["5V"] + ldo_in
    i5 = p5 / D.V_5V
    if i5 > D.BUCK_LIMIT_A:
        raise ValueError(f"5V rail {i5:.2f} A exceeds buck limit")
    buck_loss = D.buck_loss_W(i5, v_bat)
    p_in = p5 + buck_loss
    i_in = p_in / v_bat
    path = i_in ** 2 * D.PATH_R_OHM
    return dict(rail_3v3=r["3V3"], rail_5v=r["5V"], ldo_loss=D.ldo_loss_W(i_3v3), i_5v=i5,
                buck_loss=buck_loss, buck_eff=p5 / (p5 + buck_loss), path_loss=path,
                total=p_in + path, board_dissipation=D.ldo_loss_W(i_3v3) + buck_loss + path
                + r["3V3"])  # nearly all 3V3 load power is dissipated on-board


@dataclass
class PhaseBudget:
    phase: str
    hours: float
    board_W: float
    heater_batt_W: float
    heater_bay_W: float

    @property
    def total_W(self):
        return self.board_W + self.heater_batt_W + self.heater_bay_W

    @property
    def energy_Wh(self):
        return self.total_W * self.hours


def mission_budget(v_bat_nom: float = D.SERIES * 3.6) -> list:
    rows = []
    for ph in D.MISSION:
        # heater demand at the phase's mean ambient (ground phases use the cold-soak ground temperature)
        if ph.alt_start_m == ph.alt_end_m == D.GROUND_ALT_M:
            t_amb = D.GROUND_COLD_C
            p_amb = isa(D.GROUND_ALT_M).p_Pa
        else:
            # average the heater demand over the altitude sweep
            alts = [ph.alt_start_m + (ph.alt_end_m - ph.alt_start_m) * k / 20 for k in range(21)]
            t_amb = None
        board = board_input_power(ph.name, v_bat_nom)
        bay_internal = board["board_dissipation"]
        if t_amb is not None:
            hb = thermal.heater_required_W(D.BATTERY_BOX, t_amb, p_amb)
            if ph.name == "ground warm-up":
                # heater runs at its full rating until the cold-soaked pack reaches the set-point
                w = warmup(v_bat_nom)
                warm_h = min(w["minutes"] / 60.0, ph.hours)
                hb = (w["rating_W"] * warm_h + hb * (ph.hours - warm_h)) / ph.hours
            hy = thermal.heater_required_W(D.AVIONICS_BAY, t_amb, p_amb, internal_W=bay_internal)
        else:
            hb = sum(thermal.heater_required_W(D.BATTERY_BOX, isa(a, D.DESIGN_DT).T_C, isa(a).p_Pa)
                     for a in alts) / len(alts)
            hy = sum(thermal.heater_required_W(D.AVIONICS_BAY, isa(a, D.DESIGN_DT).T_C, isa(a).p_Pa,
                                               internal_W=bay_internal) for a in alts) / len(alts)
        rows.append(PhaseBudget(ph.name, ph.hours, board["total"], hb, hy))
    return rows


def warmup(v_bat_nom: float = D.SERIES * 3.6) -> dict:
    """Battery heater rating (HEATER_MARGIN x worst steady loss) and the cold-soak warm-up time."""
    cruise = isa(D.CRUISE_ALT_M, D.DESIGN_DT)
    p_g = isa(D.GROUND_ALT_M).p_Pa
    steady = max(thermal.heater_required_W(D.BATTERY_BOX, cruise.T_C, cruise.p_Pa),
                 thermal.heater_required_W(D.BATTERY_BOX, D.GROUND_COLD_C, p_g))
    rating = D.HEATER_MARGIN * steady
    heat_cap = D.PACK_CELLS_FOR_THERMAL * D.Cell().mass_g / 1000 * D.Cell().cp_J_kgK + D.BOX_HEAT_CAP_J_K
    mins = thermal.warmup_minutes(D.BATTERY_BOX, rating, D.GROUND_COLD_C, D.GROUND_COLD_C, p_g, heat_cap)
    return dict(rating_W=rating, minutes=mins, heat_capacity_J_K=heat_cap, steady_W=steady)


def capacity_fraction(t_c: float, cell: D.Cell = D.Cell()) -> float:
    pts = cell.cap_vs_T
    if t_c <= pts[0][0]:
        return pts[0][1]
    for (t0, f0), (t1, f1) in zip(pts, pts[1:]):
        if t_c <= t1:
            return f0 + (f1 - f0) * (t_c - t0) / (t1 - t0)
    return pts[-1][1]


def cell_resistance(t_c: float, cell: D.Cell = D.Cell()) -> float:
    """Arrhenius temperature dependence of DC internal resistance."""
    T = t_c + 273.15
    return cell.r25_ohm * math.exp(cell.ea_J_mol / 8.314 * (1.0 / T - 1.0 / 298.15))


def ocv(soc: float) -> float:
    soc = min(max(soc, 0.0), 1.0)
    t = D.OCV_TABLE
    for (s0, v0), (s1, v1) in zip(t, t[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return t[-1][1]


def size_pack(energy_Wh: float, cell_temp_C: float = D.BATT_TARGET_C, cell: D.Cell = D.Cell()) -> dict:
    """Smallest 4S-nP pack whose usable energy covers the mission plus reserve."""
    per_cell_Wh = cell.cap_Ah * cell.v_nom * capacity_fraction(cell_temp_C, cell) * D.EOL_CAPACITY
    needed = energy_Wh / (1.0 - D.RESERVE)
    n_p = max(1, math.ceil(needed / (per_cell_Wh * D.SERIES)))
    usable = per_cell_Wh * D.SERIES * n_p
    return dict(config=f"{D.SERIES}S{n_p}P", n_parallel=n_p, cells=D.SERIES * n_p,
                usable_Wh_eol_cold=usable, required_Wh=needed, margin_pct=100 * (usable / needed - 1),
                mass_kg=D.SERIES * n_p * cell.mass_g / 1000, rated_Wh=D.SERIES * n_p * cell.cap_Ah * cell.v_nom)
