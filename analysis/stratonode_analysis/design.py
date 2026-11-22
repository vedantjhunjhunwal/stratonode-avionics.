"""Single source of truth for the StratoNode design inputs.

Every number the analyses use lives here, with its source or the assumption behind
it. Values marked ASSUMED are engineering estimates, not measurements; the README
lists them as limitations.
"""
from dataclasses import dataclass, field

# ---------------------------------------------------------------- environment
DESIGN_DT = -13.5          # K: ISA-13.5 gives -70 C at the tropopause (JD design point)
CRUISE_ALT_M = 15500.0     # ~50,850 ft
GROUND_ALT_M = 5400.0      # high-altitude launch site (Siachen-class)
GROUND_COLD_C = -30.0      # cold-soaked aircraft at dawn on that site (ASSUMED)
GROUND_HOT_C = 50.0        # hot-day ground case for upper thermal margins (plains, summer)


@dataclass(frozen=True)
class Phase:
    name: str
    hours: float
    alt_start_m: float
    alt_end_m: float


MISSION = [
    Phase("ground warm-up", 1.0, GROUND_ALT_M, GROUND_ALT_M),
    Phase("climb", 1.5, GROUND_ALT_M, CRUISE_ALT_M),
    Phase("cruise", 7.5, CRUISE_ALT_M, CRUISE_ALT_M),
    Phase("descent", 0.5, CRUISE_ALT_M, GROUND_ALT_M),
]


# ---------------------------------------------------------------- electrical loads
@dataclass(frozen=True)
class Load:
    name: str
    rail: str            # "3V3", "5V" or "VBAT"
    p_on_W: float        # average power while enabled
    p_peak_W: float      # worst-case instantaneous power (TX burst, conversion, inrush)
    phases: tuple        # phases in which the load is enabled
    note: str


ALL = ("ground warm-up", "climb", "cruise", "descent")
LOADS = [
    Load("STM32G474 MCU @170 MHz", "3V3", 0.150, 0.200, ALL, "datasheet run-mode ~40 mA + I/O"),
    Load("sensors (2x INA226, TMP117, MS5611)", "3V3", 0.006, 0.010, ALL, "datasheets; MS5611 duty-cycled"),
    Load("status LEDs", "3V3", 0.007, 0.013, ALL, "2 mA average, PWM-dimmed"),
    Load("CAN transceiver MCP2562", "5V", 0.085, 0.250, ALL, "5 mA recessive / 45 mA dominant, 30% bus load"),
    Load("GNSS receiver (GPS+NavIC, external puck)", "5V", 0.350, 0.450, ALL, "dual-band module class, ASSUMED"),
    Load("telemetry radio (1 W class)", "5V", 1.225, 4.000, ALL, "0.3 W RX, 4 W TX at 25% TX duty, ASSUMED"),
    Load("payload port", "5V", 3.000, 4.500, ("cruise",), "3 W sensor payload, cruise only, ASSUMED"),
]

# ---------------------------------------------------------------- regulators
V_5V, V_3V3 = 5.0, 3.3
BUCK_LIMIT_A = 3.0          # TPS62933: 3 A continuous
LDO_LIMIT_A = 0.6           # AP2112K-3.3: 600 mA


def buck_loss_W(i_out: float, v_in: float) -> float:
    """TPS62933 loss model, fitted to the datasheet 12 V -> 5 V efficiency curve
    (~91-92% from 0.2 to 1 A). Switching term scales with Vin. Approximate."""
    return 0.03 + 0.30 * i_out * (v_in / 12.0) + 0.12 * i_out ** 2


def ldo_loss_W(i_out: float, v_in: float = V_5V, v_out: float = V_3V3, iq_A: float = 55e-6) -> float:
    return (v_in - v_out) * i_out + v_in * iq_A


PATH_R_OHM = 0.010 + 0.004 + 0.010   # ideal-diode FET + PCB + input shunt (INA226 #1)

# ---------------------------------------------------------------- battery
@dataclass(frozen=True)
class Cell:
    name: str = "18650 Li-ion, 3.5 Ah class (e.g. INR18650-35E)"
    cap_Ah: float = 3.35          # rated at 0.2C, 25 C
    v_nom: float = 3.6
    v_cutoff: float = 2.65        # discharge cut-off per cell
    r25_ohm: float = 0.035        # DC internal resistance at 25 C, fresh
    ea_J_mol: float = 20000.0     # Arrhenius activation energy for R(T), ASSUMED (gives ~4x at -20 C)
    mass_g: float = 48.0
    cp_J_kgK: float = 900.0
    # usable-capacity fraction vs cell temperature (C), typical 18650 data, ASSUMED
    cap_vs_T: tuple = ((-30, 0.55), (-20, 0.72), (-10, 0.85), (0, 0.93), (10, 0.97), (25, 1.0), (45, 1.0))


SERIES = 4                  # 4S: 14.4 V nominal, matches a 4S UAV avionics bus
EOL_CAPACITY = 0.80         # end-of-life capacity fraction
RESERVE = 0.20              # energy reserve kept at landing
BATT_TARGET_C = 5.0         # heater set-point for the battery box
BMS_CUTOFF_V = SERIES * 2.8  # pack under-voltage cut-off

# OCV(SOC) for one cell, typical NMC curve (SOC 0..1)
OCV_TABLE = ((0.0, 3.00), (0.05, 3.30), (0.1, 3.42), (0.2, 3.52), (0.3, 3.58), (0.4, 3.63),
             (0.5, 3.68), (0.6, 3.75), (0.7, 3.84), (0.8, 3.94), (0.9, 4.05), (1.0, 4.17))

# ---------------------------------------------------------------- enclosures (thermal)
@dataclass(frozen=True)
class Box:
    name: str
    inner_mm: tuple          # inner L, W, H
    insulation_mm: float
    k_ins: float             # W/(m K), closed-cell foam at low temperature
    emissivity: float
    target_C: float          # lowest allowed internal temperature
    internal_W: float        # internal heat generated (electronics), W


BATTERY_BOX = Box("battery box", (140.0, 80.0, 75.0), 20.0, 0.028, 0.85, BATT_TARGET_C, 0.0)
AVIONICS_BAY = Box("avionics bay", (110.0, 80.0, 40.0), 15.0, 0.028, 0.85, -25.0, 1.0)
# Bay minimum -25 C keeps a 15 K margin above the -40 C rating of the board's parts.
H_NAT_SL = 4.0     # W/(m2 K) natural-convection film coefficient at sea level, small box, ASSUMED
HEATER_MARGIN = 2.0  # heater rating / worst steady-state loss
PACK_CELLS_FOR_THERMAL = 20  # 4S5P (result of size_pack; checked in tests)
BOX_HEAT_CAP_J_K = 150.0      # cell holder, wiring and inner box wall, ASSUMED

# ---------------------------------------------------------------- holdup and brown-out
DROPOUT_S = 0.050            # longest connector micro-interruption to ride through (vibration fretting)
SHED_LATENCY_S = 0.00035     # INA226 ALERT (0.28 ms conversion cycle) -> EXTI4 -> load switches + heater PWM off
V_HOLD_START = 12.0          # pack at end of discharge under load
V_BUCK_MIN = 5.6             # TPS62933 needs ~0.6 V headroom for a regulated 5 V
N_HOLDUP_CAPS = 3             # 470 uF 25 V aluminium polymer, 8x10 mm
HOLDUP_CAP_UF = 470.0
P_ESSENTIAL_W = 0.80         # MCU + sensors + GNSS + CAN after shedding radio, payload and heaters

# ---------------------------------------------------------------- PCB (vibration / solder fatigue)
PCB = dict(len_mm=80.0, wid_mm=56.0, thick_mm=1.6, E_GPa=18.6, nu=0.12, mass_g=38.0)  # as laid out in hw/
VIB_PSD_G2_HZ = 0.04        # flat random envelope 20-2000 Hz (8.9 g rms), ASSUMED, propeller UAV
ALPHA_FR4_PPM = 17.0        # in-plane CTE of FR-4
SOLDER_FLIGHTS_TARGET = 1000  # design life in flight cycles at <=0.1% joint failures
