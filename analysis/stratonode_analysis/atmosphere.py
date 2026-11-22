"""International Standard Atmosphere (ISA) to 32 km, with a cold-day temperature offset.

Layers: troposphere (lapse -6.5 K/km to 11 km), tropopause (isothermal to 20 km),
lower stratosphere (+1.0 K/km to 32 km). Pressure is integrated with the standard
closed-form hydrostatic equations; a temperature offset (dT) shifts temperature
and density but, by convention, not the pressure-altitude relation.
"""
from dataclasses import dataclass
import math

G0 = 9.80665          # m/s^2
R_AIR = 287.053       # J/(kg K)
T0, P0 = 288.15, 101325.0
FT = 0.3048

# (base altitude m, base temperature K, lapse K/m, base pressure Pa)
_LAYERS = [
    (0.0, 288.15, -0.0065, 101325.0),
    (11000.0, 216.65, 0.0, 22632.06),
    (20000.0, 216.65, 0.001, 5474.889),
    (32000.0, 228.65, 0.0028, 868.0187),
]


@dataclass(frozen=True)
class AirState:
    altitude_m: float
    T_K: float
    p_Pa: float
    rho: float

    @property
    def T_C(self) -> float:
        return self.T_K - 273.15

    @property
    def sigma(self) -> float:
        """Density ratio rho / rho_sea_level (ISA)."""
        return self.rho / 1.225

    @property
    def mu(self) -> float:
        """Dynamic viscosity of air, Sutherland's law [Pa s]."""
        return 1.458e-6 * self.T_K ** 1.5 / (self.T_K + 110.4)

    @property
    def k_air(self) -> float:
        """Thermal conductivity of air [W/(m K)] (Sutherland-type fit)."""
        return 2.646e-3 * self.T_K ** 1.5 / (self.T_K + 245.4 * 10 ** (-12 / self.T_K))


def isa(altitude_m: float, dT: float = 0.0) -> AirState:
    if not 0.0 <= altitude_m <= 32000.0:
        raise ValueError("altitude must be within 0..32000 m")
    for i, (h0, Tb, L, pb) in enumerate(_LAYERS):
        top = _LAYERS[i + 1][0] if i + 1 < len(_LAYERS) else 47000.0
        if altitude_m <= top:
            dh = altitude_m - h0
            T = Tb + L * dh
            if L == 0.0:
                p = pb * math.exp(-G0 * dh / (R_AIR * Tb))
            else:
                p = pb * (T / Tb) ** (-G0 / (R_AIR * L))
            T_act = T + dT
            return AirState(altitude_m, T_act, p, p / (R_AIR * T_act))
    raise AssertionError("unreachable")


def isa_ft(altitude_ft: float, dT: float = 0.0) -> AirState:
    return isa(altitude_ft * FT, dT)
