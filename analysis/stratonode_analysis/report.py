"""Run every analysis and write results/analysis.json and docs/ANALYSIS.md.

    python -m stratonode_analysis.report  (from analysis/)
"""
import json
import os
import sys

from . import design as D
from . import power, thermal, failure
from .atmosphere import isa

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def run() -> dict:
    out = {}
    cruise = isa(D.CRUISE_ALT_M, D.DESIGN_DT)
    ground = isa(D.GROUND_ALT_M)
    out["environment"] = dict(cruise_alt_m=D.CRUISE_ALT_M, cruise_alt_ft=round(D.CRUISE_ALT_M / 0.3048),
                              cruise_T_C=cruise.T_C, cruise_p_kPa=cruise.p_Pa / 1000, density_ratio=cruise.sigma,
                              ground_alt_m=D.GROUND_ALT_M, ground_cold_C=D.GROUND_COLD_C)

    # ---- power budget
    vnom = D.SERIES * 3.6
    rows = power.mission_budget(vnom)
    energy = sum(r.energy_Wh for r in rows)
    out["budget"] = [dict(phase=r.phase, hours=r.hours, board_W=r.board_W, heater_batt_W=r.heater_batt_W,
                          heater_bay_W=r.heater_bay_W, total_W=r.total_W, energy_Wh=r.energy_Wh) for r in rows]
    out["mission_energy_Wh"] = energy
    cr = power.board_input_power("cruise", vnom)
    pk = power.board_input_power("cruise", vnom, peak=True)
    out["cruise_rails"] = cr
    out["peak_rails"] = pk
    out["rail_limits"] = dict(buck_A=D.BUCK_LIMIT_A, peak_5v_A=pk["i_5v"],
                              buck_current_margin_pct=100 * (1 - pk["i_5v"] / D.BUCK_LIMIT_A),
                              ldo_A=D.LDO_LIMIT_A, peak_3v3_A=pk["rail_3v3"] / D.V_3V3)
    pack = power.size_pack(energy)
    out["pack"] = pack

    # ---- thermal
    hb_cruise = thermal.heater_required_W(D.BATTERY_BOX, cruise.T_C, cruise.p_Pa)
    hb_ground = thermal.heater_required_W(D.BATTERY_BOX, D.GROUND_COLD_C, ground.p_Pa)
    hy_cruise = thermal.heater_required_W(D.AVIONICS_BAY, cruise.T_C, cruise.p_Pa, cr["board_dissipation"])
    unheated_bay = thermal.unheated_temp_C(D.AVIONICS_BAY, cruise.T_C, cruise.p_Pa, cr["board_dissipation"])
    w = power.warmup(vnom)
    hb_rating, warm, cells_heat_cap = w["rating_W"], w["minutes"], w["heat_capacity_J_K"]
    hy_rating = D.HEATER_MARGIN * max(hy_cruise, 0.5)
    out["thermal"] = dict(
        battery_box_G_W_K_cruise=thermal.conductance_W_K(D.BATTERY_BOX, cruise.T_C, cruise.p_Pa),
        battery_box_G_W_K_sea_level=thermal.conductance_W_K(D.BATTERY_BOX, cruise.T_C, 101325.0),
        battery_heater_cruise_W=hb_cruise, battery_heater_ground_W=hb_ground,
        battery_heater_rating_W=hb_rating, battery_heater_ohms_at_14V4=vnom ** 2 / hb_rating,
        bay_board_dissipation_W=cr["board_dissipation"], bay_unheated_cruise_C=unheated_bay,
        bay_heater_cruise_W=hy_cruise, bay_heater_rating_W=hy_rating,
        warmup_min_from_cold_soak=warm, pack_heat_capacity_J_K=cells_heat_cap)

    # junction temperatures: cold cruise (bay held at -25 C) and hot ground (+50 C, sea level, peak loads)
    i5_pk = pk["i_5v"]
    i33 = pk["rail_3v3"] / D.V_3V3
    loss = {"TPS62933 buck (SOT-583)": (D.buck_loss_W(cr["i_5v"], vnom), D.buck_loss_W(i5_pk, D.SERIES * 4.2)),
            "AP2112K LDO (SOT-23-5)": (D.ldo_loss_W(cr["rail_3v3"] / D.V_3V3), D.ldo_loss_W(i33)),
            "STM32G474 MCU (LQFP-48)": (0.15, 0.20),
            "heater FET AO3400A (SOT-23)": ((hb_rating / vnom) ** 2 * 0.040, (hb_rating / (D.SERIES * 4.2) * 1.0) ** 2 * 0.040)}
    jt = []
    for part, (p_cold, p_hot) in loss.items():
        d = thermal.PARTS[part]
        tc = thermal.junction_C(part, p_cold, D.AVIONICS_BAY.target_C, cruise.p_Pa)
        th = thermal.junction_C(part, p_hot, D.GROUND_HOT_C, 101325.0)
        jt.append(dict(part=part, p_cold_W=p_cold, tj_cold_C=tc, p_hot_W=p_hot, tj_hot_C=th,
                       rating=f"{d['t_min']}..{d['t_max']} C", margin_cold_K=tc - d["t_min"],
                       margin_hot_K=d["t_max"] - th))
    out["junctions"] = jt

    # ---- brown-out
    n_p = pack["n_parallel"]
    i_step = failure.worst_step_current(D.SERIES * 3.3, hb_rating + hy_rating)
    out["brownout"] = dict(step_current_A=i_step, cases=[
        dict(case="heated pack, end of flight", **failure.pack_sag(D.BATT_TARGET_C, 0.15, i_step, n_p)),
        dict(case="cold-soaked start, 50% SOC", **failure.pack_sag(D.GROUND_COLD_C, 0.5, i_step, n_p)),
        dict(case="heater failed at cruise, 15% SOC", **failure.pack_sag(-40.0, 0.15, i_step, n_p)),
    ])

    # ---- hold-up
    p_full = pk["total"] + hb_rating + hy_rating   # heaters hang off VBAT too and must be shed
    p_ess = D.P_ESSENTIAL_W / 0.85
    c_shed = failure.holdup_capacitance_F(p_full, p_ess, True)
    c_noshed = failure.holdup_capacitance_F(p_full, p_ess, False)
    i_in = p_full / D.V_HOLD_START
    out["holdup"] = dict(p_full_W=p_full, p_essential_W=p_ess, dropout_ms=1000 * D.DROPOUT_S,
                         c_required_uF_with_shedding=1e6 * c_shed, c_required_uF_without=1e6 * c_noshed,
                         chosen=f"{D.N_HOLDUP_CAPS} x {D.HOLDUP_CAP_UF:.0f} uF 25 V aluminium polymer",
                         chosen_uF_cold=D.N_HOLDUP_CAPS * D.HOLDUP_CAP_UF * failure.CAP_COLD["aluminium polymer"]["c_ratio"],
                         esr_step_mV_polymer_cold=1000 * failure.esr_step_V("aluminium polymer", i_in, D.N_HOLDUP_CAPS),
                         esr_step_mV_electrolytic_cold=1000 * failure.esr_step_V("aluminium electrolytic", i_in, D.N_HOLDUP_CAPS))

    # ---- solder fatigue
    sol = []
    for pkg in failure.PACKAGES:
        heated = failure.solder_life(pkg, 25.0, D.AVIONICS_BAY.target_C)
        unheated = failure.solder_life(pkg, 25.0, unheated_bay)
        sol.append(dict(package=pkg, n_0p1pct_heated=heated["n_0p1pct"], n_0p1pct_unheated=unheated["n_0p1pct"],
                        ok_heated=heated["n_0p1pct"] >= D.SOLDER_FLIGHTS_TARGET,
                        ok_unheated=unheated["n_0p1pct"] >= D.SOLDER_FLIGHTS_TARGET))
    out["solder"] = dict(target_flights=D.SOLDER_FLIGHTS_TARGET, heated_swing=f"25 to {D.AVIONICS_BAY.target_C} C",
                         unheated_swing=f"25 to {unheated_bay:.0f} C", rows=sol)

    # ---- vibration
    out["vibration"] = dict(psd_g2_Hz=D.VIB_PSD_G2_HZ, rows=[
        dict(component="LQFP-48 MCU at board centre", **failure.steinberg(9.0, 1.0)),
        dict(component="SOIC-8 CAN transceiver", **failure.steinberg(4.9, 1.0)),
        dict(component="1206 shunt (leadless, c=2.25)", **failure.steinberg(3.2, 2.25)),
    ])
    return out


def _f(x, n=2):
    return f"{x:.{n}f}"


def markdown(r: dict) -> str:
    e = r["environment"]
    L = ["# StratoNode design analysis", "",
         "Generated by `python -m stratonode_analysis.report`. Every input is in `design.py`; ",
         "values marked ASSUMED there are engineering estimates, not measurements.", "",
         f"Design point: {e['cruise_alt_m']:.0f} m ({e['cruise_alt_ft']} ft), {e['cruise_T_C']:.1f} C, "
         f"{e['cruise_p_kPa']:.2f} kPa, air density {100 * e['density_ratio']:.1f}% of sea level. "
         f"Launch site {e['ground_alt_m']:.0f} m, cold-soaked at {e['ground_cold_C']:.0f} C.", "",
         f"## 1. Power budget ({sum(b['hours'] for b in r['budget']):g} h mission including pre-flight warm-up, 4S pack at 14.4 V)", "",
         "| Phase | Hours | Board in (W) | Battery heater (W) | Bay heater (W) | Total (W) | Energy (Wh) |",
         "|---|---|---|---|---|---|---|"]
    for b in r["budget"]:
        L.append(f"| {b['phase']} | {b['hours']} | {_f(b['board_W'])} | {_f(b['heater_batt_W'])} | "
                 f"{_f(b['heater_bay_W'])} | {_f(b['total_W'])} | {_f(b['energy_Wh'], 1)} |")
    p, lim, cr = r["pack"], r["rail_limits"], r["cruise_rails"]
    L += ["", f"Mission energy **{_f(r['mission_energy_Wh'], 1)} Wh**. Cruise: buck efficiency "
          f"{100 * cr['buck_eff']:.1f}%, on-board dissipation {_f(cr['board_dissipation'])} W.",
          f"Peak 5 V rail current {_f(lim['peak_5v_A'])} A against the 3 A buck rating "
          f"({lim['buck_current_margin_pct']:.0f}% margin); peak 3V3 {1000 * lim['peak_3v3_A']:.0f} mA against 600 mA.", "",
          f"**Pack: {p['config']}** ({p['cells']} cells, {p['mass_kg']:.2f} kg, {p['rated_Wh']:.0f} Wh rated). "
          f"Usable at end of life, at the 5 C heater set-point: {_f(p['usable_Wh_eol_cold'], 1)} Wh against "
          f"{_f(p['required_Wh'], 1)} Wh needed with a 20% reserve ({p['margin_pct']:.0f}% margin).", "",
          "## 2. Thermal", ""]
    t = r["thermal"]
    L += [f"- Battery box conductance: {_f(t['battery_box_G_W_K_cruise'], 3)} W/K at cruise vs "
          f"{_f(t['battery_box_G_W_K_sea_level'], 3)} W/K at sea-level pressure (thin air removes convection; radiation remains).",
          f"- Battery heater: {_f(t['battery_heater_cruise_W'])} W steady at cruise, {_f(t['battery_heater_ground_W'])} W on the cold ground. "
          f"Rated {_f(t['battery_heater_rating_W'], 1)} W (2x margin) = {_f(t['battery_heater_ohms_at_14V4'], 1)} ohm at 14.4 V; "
          f"warms the cold-soaked pack from -30 C to +5 C in {_f(t['warmup_min_from_cold_soak'], 0)} min.",
          f"- Avionics bay: the board dissipates {_f(t['bay_board_dissipation_W'])} W; unheated it would settle at "
          f"**{_f(t['bay_unheated_cruise_C'], 1)} C**, below the -40 C rating of most parts. A "
          f"{_f(t['bay_heater_cruise_W'])} W bay heater holds -25 C (heater rated {_f(t['bay_heater_rating_W'], 1)} W).", "",
          "| Part | Cold cruise: P (W) | Tj (C) | Hot ground +50 C: P (W) | Tj (C) | Rating | Margins (K) |",
          "|---|---|---|---|---|---|---|"]
    for j in r["junctions"]:
        L.append(f"| {j['part']} | {_f(j['p_cold_W'], 3)} | {_f(j['tj_cold_C'], 1)} | {_f(j['p_hot_W'], 3)} | "
                 f"{_f(j['tj_hot_C'], 1)} | {j['rating']} | {_f(j['margin_cold_K'], 0)} / {_f(j['margin_hot_K'], 0)} |")
    b = r["brownout"]
    L += ["", "## 3. Brown-out and hold-up", "",
          f"Worst load step (all loads at peak, both heaters on): {_f(b['step_current_A'])} A from the pack.", "",
          "| Case | Cell R (mOhm) | Pack R (mOhm) | OCV (V) | Terminal (V) | Margin to BMS cut-off (V) | Margin to buck minimum (V) |",
          "|---|---|---|---|---|---|---|"]
    for c in b["cases"]:
        L.append(f"| {c['case']} | {_f(c['r_cell_mohm'], 0)} | {_f(c['r_pack_mohm'], 0)} | {_f(c['v_ocv'])} | "
                 f"{_f(c['v_terminal'])} | {_f(c['margin_to_bms_V'])} | {_f(c['margin_to_buck_V'])} |")
    h = r["holdup"]
    L += ["", f"Load at the moment of dropout, heaters included: {h['p_full_W']:.1f} W. Riding through a {h['dropout_ms']:.0f} ms input dropout (connector fretting under vibration) from 12 V "
          f"down to the buck's 5.6 V minimum needs **{h['c_required_uF_without']:.0f} uF** if every load stays on, "
          f"but only **{h['c_required_uF_with_shedding']:.0f} uF** when firmware sheds the radio, payload and both heaters within 0.35 ms (INA226 ALERT -> EXTI interrupt). "
          f"Fitted: {h['chosen']} ({h['chosen_uF_cold']:.0f} uF at -55 C, "
          f"{100 * (h['chosen_uF_cold'] / h['c_required_uF_with_shedding'] - 1):.0f}% margin). ",
          f"At -55 C the step caused by capacitor ESR is {h['esr_step_mV_polymer_cold']:.0f} mV with polymer parts "
          f"vs {h['esr_step_mV_electrolytic_cold']:.0f} mV with standard electrolytics, which is why the bulk capacitors are polymer.",
          "", "## 4. Solder-joint thermal-cycling fatigue (Engelmaier)", "",
          f"Flight cycles to 0.1% joint failures (Weibull slope 4), target {r['solder']['target_flights']} flights. "
          f"Heated bay swings {r['solder']['heated_swing']}; unheated {r['solder']['unheated_swing']}.", "",
          "| Package | Heated bay | Unheated |", "|---|---|---|"]
    for s in r["solder"]["rows"]:
        fmt = lambda n, ok: (f"{n:,.0f}" if n < 1e7 else ">10M") + ("" if ok else " **FAILS**")
        L.append(f"| {s['package']} | {fmt(s['n_0p1pct_heated'], s['ok_heated'])} | {fmt(s['n_0p1pct_unheated'], s['ok_unheated'])} |")
    v = r["vibration"]
    L += ["", "Result: 2512 shunts fail the target even with the bay heated, so the board uses 1206 shunts; "
          "the bay heater is what keeps 1206 and 1210 parts above target.", "",
          "## 5. Random vibration (Steinberg)", "",
          f"Input: flat {v['psd_g2_Hz']} g2/Hz, 20-2000 Hz (ASSUMED envelope). Board 80 x 56 x 1.6 mm, 38 g populated (ASSUMED), 6-point mount.", "",
          "| Component | Board fn (Hz) | Response (g rms) | 3-sigma deflection (mm) | Allowable (mm) | Margin |",
          "|---|---|---|---|---|---|"]
    for x in v["rows"]:
        L.append(f"| {x['component']} | {_f(x['fn_Hz'], 0)} | {_f(x['g_rms_response'], 1)} | {_f(x['z3sigma_mm'], 4)} | "
                 f"{_f(x['z_allow_mm'], 3)} | {_f(x['margin'], 1)}x |")
    return "\n".join(L) + "\n"


def main():
    r = run()
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", "analysis.json"), "w") as f:
        json.dump(r, f, indent=2, default=float)
    md = markdown(r)
    with open(os.path.join(ROOT, "docs", "ANALYSIS.md"), "w") as f:
        f.write(md)
    sys.stdout.write(md)


if __name__ == "__main__":
    main()
