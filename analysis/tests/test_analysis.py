import math
import pytest

from stratonode_analysis import design as D, power, thermal, failure
from stratonode_analysis.atmosphere import isa, isa_ft
from stratonode_analysis.report import run


def test_isa_reference_points():
    assert isa(0).T_K == pytest.approx(288.15)
    assert isa(0).rho == pytest.approx(1.225, rel=1e-3)
    assert isa(11000).p_Pa == pytest.approx(22632, rel=1e-3)
    assert isa(20000).p_Pa == pytest.approx(5474.9, rel=1e-3)
    # 50,000 ft: ~11.6 kPa, ~15% of sea-level density (JD figure)
    s = isa_ft(50000)
    assert s.p_Pa == pytest.approx(11597, rel=5e-3)
    assert 0.14 < s.sigma < 0.16


def test_design_cold_day_gives_minus_70():
    assert isa(D.CRUISE_ALT_M, D.DESIGN_DT).T_C == pytest.approx(-70.0, abs=0.01)


def test_cell_resistance_rises_in_cold():
    r25, r_m20 = power.cell_resistance(25), power.cell_resistance(-20)
    assert r25 == pytest.approx(D.Cell().r25_ohm)
    assert 3.0 < r_m20 / r25 < 5.0


def test_buck_budget_energy_balance():
    b = power.board_input_power("cruise", 14.4)
    out = b["rail_3v3"] + b["rail_5v"]
    assert b["total"] == pytest.approx(out + b["ldo_loss"] + b["buck_loss"] + b["path_loss"] + 14.4 * 0 + 5.0 * 55e-6 * 0, rel=1e-2)
    assert 0.85 < b["buck_eff"] < 0.95


def test_pack_sizing_is_minimal_and_sufficient():
    e = sum(r.energy_Wh for r in power.mission_budget())
    p = power.size_pack(e)
    assert p["usable_Wh_eol_cold"] >= p["required_Wh"]
    smaller = (p["n_parallel"] - 1) * p["usable_Wh_eol_cold"] / p["n_parallel"]
    assert smaller < p["required_Wh"]
    assert p["cells"] == D.PACK_CELLS_FOR_THERMAL   # thermal mass assumption matches the sized pack


def test_heat_loss_monotonic_and_thin_air_reduces_convection():
    box = D.BATTERY_BOX
    q1 = thermal.heat_loss_W(box, 5, -50, 11000)
    q2 = thermal.heat_loss_W(box, 5, -70, 11000)
    assert q2 > q1 > 0
    assert thermal.conductance_W_K(box, -70, 11000) < thermal.conductance_W_K(box, -70, 101325)


def test_heater_zero_when_warm_enough():
    assert thermal.heater_required_W(D.BATTERY_BOX, 20, 101325) == 0.0


def test_unheated_temperature_consistent_with_heat_loss():
    t = thermal.unheated_temp_C(D.AVIONICS_BAY, -70, 11000, 1.0)
    assert thermal.heat_loss_W(D.AVIONICS_BAY, t, -70, 11000) == pytest.approx(1.0, rel=1e-4)


def test_engelmaier_hand_calculation():
    # 1206, 25 -> -55 C, 1 cycle/day: hand-computed median ~5,000 cycles
    r = failure.solder_life("1206 current shunt", 25, -55)
    assert r["strain"] == pytest.approx(1.6 * 10.5e-6 * 80 / 0.10)
    assert 4000 < r["n50"] < 6500
    # bigger parts and bigger swings fail sooner
    assert failure.solder_life("2512 current shunt", 25, -55)["n50"] < r["n50"]
    assert failure.solder_life("1206 current shunt", 25, -25)["n50"] > r["n50"]


def test_holdup_shedding_cuts_capacitance_about_tenfold():
    full, ess = 12.0, 1.0
    c_shed = failure.holdup_capacitance_F(full, ess, True)
    c_none = failure.holdup_capacitance_F(full, ess, False)
    assert 5 < c_none / c_shed < 15
    # energy check
    e = 0.5 * c_none * (D.V_HOLD_START ** 2 - D.V_BUCK_MIN ** 2)
    assert e == pytest.approx(full * D.DROPOUT_S)


def test_plate_frequency_hand_calculation():
    fn = failure.plate_fn_Hz()
    assert 550 < fn < 750


def test_full_report_margins_positive():
    r = run()
    assert r["rail_limits"]["peak_5v_A"] < D.BUCK_LIMIT_A
    assert r["holdup"]["chosen_uF_cold"] > r["holdup"]["c_required_uF_with_shedding"]
    assert all(j["margin_cold_K"] > 0 and j["margin_hot_K"] > 0 for j in r["junctions"])
    assert all(c["margin_to_bms_V"] > 0 for c in r["brownout"]["cases"])
    assert all(v["margin"] > 1 for v in r["vibration"]["rows"])
    assert r["thermal"]["warmup_min_from_cold_soak"] <= 60
