"""StratoNode v0.1 netlist: every component, its footprint, part number and pin-to-net map.

This file is the design source. gen_schematic.py turns it into a KiCad schematic; the
PCB is built from the netlist KiCad exports from that schematic.
"""

R0603 = "Resistor_SMD:R_0603_1608Metric"
R1206 = "Resistor_SMD:R_1206_3216Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
GH = "Connector_JST:JST_GH_SM0{n}B-GHS-TB_1x0{n}-1MP_P1.25mm_Horizontal"

PARTS = []


def P(ref, lib, value, fp, block, pins, mpn="", note=""):
    PARTS.append(dict(ref=ref, lib=lib, value=value, fp=fp, block=block, pins=pins, mpn=mpn, note=note))


def R(ref, value, a, b, block, fp=R0603, mpn="", note=""):
    P(ref, "Device:R", value, fp, block, {"1": a, "2": b}, mpn or f"{value} 1% 0603, AEC-Q200, -55..155 C", note)


def C(ref, value, a, b, block, fp=C0603, mpn="", note=""):
    P(ref, "Device:C", value, fp, block, {"1": a, "2": b}, mpn or f"{value} X7R, -55..125 C", note)


# ------------------------------------------------------------------ 1. input, protection, hold-up
B1 = "1  Power input, protection, hold-up"
P("J1", "Connector_Generic:Conn_01x02", "XT30 4S in", "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal", B1,
  {"1": "GND", "2": "VIN"}, "AMASS XT30PW-M", "locking-friction power connector; 4S pack 12.0-16.8 V")
P("D1", "Device:D_TVS", "SMBJ18CA", "Diode_SMD:D_SMB", B1, {"1": "VIN", "2": "GND"}, "SMBJ18CA",
  "bidirectional TVS: clamps load dump below the buck's 30 V rating; blocks nothing")
R("R1", "10m", "VIN", "VIN_S", B1, fp=R1206, mpn="10 mOhm 1206 0.5 W current shunt", note="1206 not 2512: solder-fatigue analysis")
P("U1", "Power_Management:LM74700", "LM74700-Q1", "Package_TO_SOT_SMD:SOT-23-6", B1,
  {"1": "IDEAL_VCAP", "2": "GND", "3": "VIN_S", "4": "VBAT", "5": "IDEAL_GATE", "6": "VIN_S"}, "LM74700QDBVRQ1",
  "ideal-diode controller: reverse-polarity block, and stops the hold-up caps back-feeding a dropped input")
C("C1", "100n", "IDEAL_VCAP", "VIN_S", B1)
P("Q1", "Device:Q_NMOS_GDS", "BUK7Y4R4-40H", "Package_TO_SOT_SMD:TO-252-2", B1,
  {"1": "IDEAL_GATE", "2": "VBAT", "3": "VIN_S"}, "40 V, 4.4 mOhm N-FET (ideal diode)")
for i, ref in enumerate(("C2", "C3", "C4")):
    P(ref, "Device:C_Polarized", "470u 25V polymer", "Capacitor_SMD:CP_Elec_8x10", B1, {"1": "VBAT", "2": "GND"},
      "25 V 470 uF aluminium polymer, -55..105 C", "hold-up: 3 x 470 uF rides a 50 ms dropout with load shedding")
P("U4", "Sensor_Energy:INA226", "INA226 0x40", "Package_SO:VSSOP-10_3x3mm_P0.5mm", B1,
  {"1": "GND", "2": "GND", "3": "PWR_FAIL_N", "4": "I2C1_SDA", "5": "I2C1_SCL", "6": "+3V3", "7": "GND",
   "8": "VIN", "9": "VIN_S", "10": "VIN"}, "INA226AIDGSR", "input V/I monitor; ALERT = bus under-voltage -> load shedding")
C("C5", "100n", "+3V3", "GND", B1)
R("R2", "100k", "VIN", "VIN_SENSE", B1)
R("R3", "10k", "VIN_SENSE", "GND", B1)
C("C6", "100n", "VIN_SENSE", "GND", B1, note="ADC analog-watchdog input: fast dropout detection")

# ------------------------------------------------------------------ 2. 5 V buck and 3V3 LDO
B2 = "2  5 V buck (3 A) and 3V3 LDO"
P("U2", "Regulator_Switching:TPS62933", "TPS62933", "Package_TO_SOT_SMD:SOT-583-8", B2,
  {"1": "BUCK_RT", "2": "BUCK_EN", "3": "VBAT", "4": "GND", "5": "BUCK_SW", "6": "BUCK_BST", "7": "BUCK_SS", "8": "BUCK_FB"},
  "TPS62933DRLR", "3.8-30 V in, 3 A synchronous buck, -40..150 C junction")
R("R4", "470k", "VBAT", "BUCK_EN", B2, note="EN divider: turns on above ~6.9 V")
R("R5", "100k", "BUCK_EN", "GND", B2)
R("R6", "100k", "BUCK_RT", "GND", B2, note="sets switching frequency; check value against datasheet RT table")
C("C7", "10u 25V", "VBAT", "GND", B2, fp=C0805)
C("C8", "10u 25V", "VBAT", "GND", B2, fp=C0805)
C("C9", "100n", "VBAT", "GND", B2)
C("C10", "100n", "BUCK_BST", "BUCK_SW", B2)
C("C11", "22n", "BUCK_SS", "GND", B2)
P("L1", "Device:L", "6.8u", "Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm", B2, {"1": "BUCK_SW", "2": "+5V"},
  "Bourns SRP7028A-6R8M", "6.8 uH, Isat > 5 A")
R("R7", "100k", "+5V", "BUCK_FB", B2, note="FB divider: 0.8 V x (1 + 100k/19.1k) = 5.0 V")
R("R8", "19.1k", "BUCK_FB", "GND", B2)
C("C12", "22u 10V", "+5V", "GND", B2, fp=C0805)
C("C13", "22u 10V", "+5V", "GND", B2, fp=C0805)
C("C14", "22u 10V", "+5V", "GND", B2, fp=C0805)
P("U3", "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5", B2,
  {"1": "+5V", "2": "GND", "3": "+5V", "4": None, "5": "+3V3"}, "AP2112K-3.3TRG1", "600 mA LDO, -40..85 C ambient")
C("C15", "1u", "+5V", "GND", B2)
C("C16", "4.7u", "+3V3", "GND", B2, fp=C0805)

# ------------------------------------------------------------------ 3. switched outputs and heaters
B3 = "3  Load switches (shedding) and heater drivers"
for ref, en, out, ct in (("U6", "RADIO_EN", "+5V_RADIO", "RADIO_CT"), ("U7", "PAYLOAD_EN", "+5V_PAYLOAD", "PAYLOAD_CT")):
    P(ref, "Power_Management:TPS22917DBV", "TPS22917", "Package_TO_SOT_SMD:SOT-23-6", B3,
      {"1": "+5V", "2": "GND", "3": en, "4": ct, "5": out, "6": out}, "TPS22917DBVR",
      "2 A load switch, ~1 us turn-off: firmware sheds this rail on power-fail")
C("C17", "1n", "RADIO_CT", "GND", B3, note="slew rate: limits inrush from radio input caps")
C("C18", "1n", "PAYLOAD_CT", "GND", B3)
R("R9", "100k", "RADIO_EN", "GND", B3, note="pull-downs: outputs default OFF during reset/brown-out")
R("R10", "100k", "PAYLOAD_EN", "GND", B3)
C("C19", "10u", "+5V_RADIO", "GND", B3, fp=C0805)
C("C20", "10u", "+5V_PAYLOAD", "GND", B3, fp=C0805)
R("R11", "10m", "VBAT", "VBAT_HTR", B3, fp=R1206, mpn="10 mOhm 1206 0.5 W current shunt")
P("U8", "Sensor_Energy:INA226", "INA226 0x41", "Package_SO:VSSOP-10_3x3mm_P0.5mm", B3,
  {"1": "GND", "2": "+3V3", "3": "HTR_ALERT_N", "4": "I2C1_SDA", "5": "I2C1_SCL", "6": "+3V3", "7": "GND",
   "8": "VBAT_HTR", "9": "VBAT_HTR", "10": "VBAT"}, "INA226AIDGSR", "heater current: detects open/shorted heater element")
C("C21", "100n", "+3V3", "GND", B3)
for q, rg, rp, pwm, lo, j in (("Q2", "R12", "R13", "HTR1_PWM", "HTR1_LO", "J8"), ("Q3", "R14", "R15", "HTR2_PWM", "HTR2_LO", "J9")):
    P(q, "Device:Q_NMOS_GSD", "AO3400A", "Package_TO_SOT_SMD:SOT-23", B3,
      {"1": pwm + "_G", "2": "GND", "3": lo}, "AO3400A", "30 V 5.7 A logic-level N-FET, low-side heater switch")
    R(rg, "100", pwm, pwm + "_G", B3)
    R(rp, "100k", pwm + "_G", "GND", B3, note="heater OFF if the MCU is in reset")
    P(j, "Connector_Generic:Conn_01x02", "heater " + ("battery" if j == "J8" else "bay"),
      "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", B3, {"1": "VBAT_HTR", "2": lo}, "JST B2B-XH-A",
      "3 A contacts; heater element 16 ohm (battery) / 47 ohm (bay)")
P("D2", "Device:D_Schottky", "SS34", "Diode_SMD:D_SMA", B3, {"1": "VBAT_HTR", "2": "HTR1_LO"}, "SS34",
  "clamps harness-inductance kickback at heater switch-off")
P("D3", "Device:D_Schottky", "SS34", "Diode_SMD:D_SMA", B3, {"1": "VBAT_HTR", "2": "HTR2_LO"}, "SS34")

# ------------------------------------------------------------------ 4. MCU, clock, debug, LEDs
B4 = "4  STM32G474 MCU, clock, debug"
mcu = {"1": "+3V3", "2": "LED_STATUS", "3": "LED_FAULT", "4": "HTR_ALERT_N", "5": "OSC_IN", "6": "OSC_OUT",
       "7": "NRST", "8": "TEL_CTS", "9": "TEL_RTS", "10": "TEL_TX", "11": "TEL_RX", "12": "BARO_CS",
       "13": "SPI1_SCK", "14": "SPI1_MISO", "15": "SPI1_MOSI", "16": "HTR1_PWM", "17": "HTR2_PWM",
       "18": "VIN_SENSE", "19": "GND", "20": "+3V3A", "21": "+3V3A", "22": "PAY_TX", "23": "GND", "24": "+3V3",
       "25": "PAY_RX", "26": "NTC_BATT", "27": "RADIO_EN", "28": "PAYLOAD_EN", "29": "CAN_STBY",
       "30": "I2C2_SDA", "31": "I2C2_SCL", "32": "GNSS_RX", "33": "CAN_RXD", "34": "CAN_TXD", "35": "GND",
       "36": "+3V3", "37": "SWDIO", "38": "SWCLK", "39": "I2C1_SCL", "40": "SWO", "41": "PWR_FAIL_N",
       "42": None, "43": "GNSS_TX", "44": "I2C1_SDA", "45": "BOOT0", "46": None, "47": "GND", "48": "+3V3"}
P("U5", "MCU_ST_STM32G4:STM32G474C_B-C-E_Tx", "STM32G474CET3", "Package_QFP:LQFP-48_7x7mm_P0.5mm", B4, mcu,
  "STM32G474CET3", "170 MHz Cortex-M4F, FDCAN, -40..125 C grade")
for k, ref in enumerate(("C22", "C23", "C24", "C25")):
    C(ref, "100n", "+3V3", "GND", B4)
C("C26", "4.7u", "+3V3", "GND", B4, fp=C0805)
P("FB1", "Device:FerriteBead_Small", "600R@100MHz", "Inductor_SMD:L_0603_1608Metric", B4, {"1": "+3V3", "2": "+3V3A"},
  "BLM18KG601SN1D", "isolates the ADC reference from digital noise")
C("C27", "1u", "+3V3A", "GND", B4)
C("C28", "100n", "+3V3A", "GND", B4)
P("Y1", "Device:Crystal_GND24", "16MHz", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", B4,
  {"1": "OSC_IN", "2": "GND", "3": "OSC_OUT", "4": "GND"}, "16 MHz 10 pF, -40..125 C",
  "crystal, not RC: FDCAN bit timing needs <0.5% clock error at -40 C")
C("C29", "12p", "OSC_IN", "GND", B4, mpn="12 pF C0G")
C("C30", "12p", "OSC_OUT", "GND", B4, mpn="12 pF C0G")
C("C31", "100n", "NRST", "GND", B4)
R("R16", "10k", "BOOT0", "GND", B4)
P("J2", "Connector:Conn_ARM_SWD_TagConnect_TC2030-NL", "SWD", "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical", B4,
  {"1": "+3V3", "2": "SWDIO", "3": "NRST", "4": "SWCLK", "5": "GND", "6": "SWO"}, "Tag-Connect TC2030-NL",
  "no connector fitted: nothing to vibrate loose in flight")
P("D4", "Device:LED", "green", "LED_SMD:LED_0603_1608Metric", B4, {"1": "LED_STATUS_K", "2": "+3V3"}, "green 0603 LED")
P("D5", "Device:LED", "red", "LED_SMD:LED_0603_1608Metric", B4, {"1": "LED_FAULT_K", "2": "+3V3"}, "red 0603 LED")
R("R17", "1k", "LED_STATUS", "LED_STATUS_K", B4)
R("R18", "1k", "LED_FAULT", "LED_FAULT_K", B4)

# ------------------------------------------------------------------ 5. sensors and internal I2C
B5 = "5  Sensors: baro (SPI), board temp, battery NTC"
P("U9", "Sensor_Pressure:MS5611-01BA", "MS5611", "Package_LGA:LGA-8_3x5mm_P1.25mm", B5,
  {"1": "+3V3", "2": "GND", "3": "GND", "4": "BARO_CS", "5": "BARO_CS", "6": "SPI1_MISO", "7": "SPI1_MOSI", "8": "SPI1_SCK"},
  "MS561101BA03-50", "10-1200 mbar: covers 50,000 ft (~116 mbar); PS low = SPI")
C("C32", "100n", "+3V3", "GND", B5)
P("U10", "Sensor_Temperature:TMP117xxDRV", "TMP117", "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm", B5,
  {"1": "I2C1_SCL", "2": "GND", "3": None, "4": "GND", "5": "+3V3", "6": "I2C1_SDA"}, "TMP117MAIDRVR",
  "+/-0.1 C board temperature, 0x48; feeds the bay heater loop")
C("C33", "100n", "+3V3", "GND", B5)
R("R19", "2.2k", "I2C1_SCL", "+3V3", B5, note="I2C1 internal bus pull-ups")
R("R20", "2.2k", "I2C1_SDA", "+3V3", B5)
R("R21", "10k", "PWR_FAIL_N", "+3V3", B5)
R("R22", "10k", "HTR_ALERT_N", "+3V3", B5)
P("J10", "Connector_Generic:Conn_01x02", "battery NTC", GH.format(n=2), B5, {"1": "NTC_BATT", "2": "GND"},
  "JST SM02B-GHS-TB", "10k NTC on the cells")
R("R23", "10k", "+3V3A", "NTC_BATT", B5, note="NTC divider, 0.1%")
C("C34", "100n", "NTC_BATT", "GND", B5)

# ------------------------------------------------------------------ 6. CAN and external connectors
B6 = "6  CAN-FD, GNSS (GPS+NavIC), telemetry, payload"
P("U11", "Interface_CAN_LIN:MCP2562-E-SN", "MCP2562", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", B6,
  {"1": "CAN_TXD", "2": "GND", "3": "+5V", "4": "CAN_RXD", "5": "+3V3", "6": "CANL", "7": "CANH", "8": "CAN_STBY"},
  "MCP2562FD-E/SN", "CAN-FD transceiver, 5 V bus side, 3.3 V logic (Vio)")
C("C35", "100n", "+5V", "GND", B6)
C("C36", "100n", "+3V3", "GND", B6)
R("R24", "10k", "CAN_STBY", "+3V3", B6, note="transceiver in standby (silent) until firmware enables it")
P("JP1", "Jumper:SolderJumper_2_Open", "CAN term", "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm", B6,
  {"1": "CANH", "2": "CAN_TERM"}, "", "close on the node at the end of the bus")
R("R25", "60.4", "CAN_TERM", "CAN_SPLIT", B6, note="split termination 2 x 60.4 ohm + 4.7 nF: EMI common-mode filter")
R("R26", "60.4", "CAN_SPLIT", "CANL", B6)
C("C37", "4.7n", "CAN_SPLIT", "GND", B6)
for j in ("J3", "J4"):
    P(j, "Connector_Generic:Conn_01x04", "CAN", GH.format(n=4), B6, {"1": "+5V", "2": "CANH", "3": "CANL", "4": "GND"},
      "JST SM04B-GHS-TB", "Pixhawk-standard CAN pinout, daisy-chain in/out; GH has a positive latch")
P("J5", "Connector_Generic:Conn_01x06", "GNSS", GH.format(n=6), B6,
  {"1": "+5V", "2": "GNSS_TX", "3": "GNSS_RX", "4": "I2C2_SCL", "5": "I2C2_SDA", "6": "GND"}, "JST SM06B-GHS-TB",
  "external GNSS (GPS+NavIC) + compass; its I2C is a separate bus so a harness fault can't hang internal sensors")
R("R27", "2.2k", "I2C2_SCL", "+3V3", B6)
R("R28", "2.2k", "I2C2_SDA", "+3V3", B6)
P("J6", "Connector_Generic:Conn_01x06", "TELEM", GH.format(n=6), B6,
  {"1": "+5V_RADIO", "2": "TEL_TX", "3": "TEL_RX", "4": "TEL_CTS", "5": "TEL_RTS", "6": "GND"}, "JST SM06B-GHS-TB",
  "telemetry radio, hardware flow control")
P("J7", "Connector_Generic:Conn_01x04", "PAYLOAD", GH.format(n=4), B6,
  {"1": "+5V_PAYLOAD", "2": "PAY_TX", "3": "PAY_RX", "4": "GND"}, "JST SM04B-GHS-TB")

# mechanical: 6-point mount (vibration analysis assumes it)
for k in range(6):
    P(f"H{k + 1}", "Mechanical:MountingHole", "M2.5", "MountingHole:MountingHole_2.7mm_M2.5", "7  Mechanical", {}, "M2.5 standoff")
