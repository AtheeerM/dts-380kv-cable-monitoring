"""
DTS Cable Temperature Simulation - Thermal Model (IEC 60287 based)
===================================================================
Steady-state thermal model of a 380 kV XLPE cable buried directly in
soil, following the structure of IEC 60287-1-1 (current rating
equations) and IEC 60287-2-1 (thermal resistance calculations).

The model computes conductor temperature as:

    theta_c = theta_amb + dtheta

where the temperature rise above ambient is driven by conductor
losses, dielectric losses, sheath losses, and the series chain of
thermal resistances between conductor and soil:

    dtheta = (Wc + 0.5*Wd) * T1
           + (Wc*(1+l1) + Wd) * n * T2
           + (Wc*(1+l1+l2) + Wd) * n * (T3 + T4)

Symbols follow IEC 60287 notation. All parameters are declared
explicitly below so they can be defended in a report or viva.

NOTE ON SCOPE
-------------
This is a steady-state model. Real DTS systems observe transient
behaviour too (load cycling, emergency ratings). Transients would
require IEC 60853. The steady-state case is sufficient to demonstrate
hotspot detection and alarm behaviour, which is the purpose here.
"""

from dataclasses import dataclass, field
import numpy as np


# =====================================================================
# 1. CABLE AND INSTALLATION PARAMETERS
# =====================================================================
# Representative values for a 380 kV XLPE single-core cable,
# 2000 mm^2 copper conductor, directly buried in trefoil formation.
# These are typical published values, not manufacturer data. If you
# obtain a real datasheet from National Grid SA, replace them here and
# the rest of the model stays valid.

@dataclass
class CableParameters:
    # --- Electrical ---
    U0: float = 220_000.0        # V, phase-to-earth voltage (380/sqrt(3))
    frequency: float = 60.0      # Hz, Saudi grid frequency
    rated_current: float = 1200.0  # A, rated continuous current
    # Calibrated so that at rated load in normal soil (rho = 1.0) the
    # conductor settles near 59 C, giving sensible headroom below both
    # the 70 C alarm setpoint and the 90 C XLPE continuous limit.

    # Conductor
    conductor_area_mm2: float = 2000.0
    rho20_copper: float = 1.7241e-8   # ohm*m, copper resistivity at 20 C
    alpha20_copper: float = 3.93e-3   # 1/K, temperature coefficient
    ks: float = 1.0              # skin effect coefficient (IEC Table 2)
    kp: float = 0.8              # proximity effect coefficient

    # --- Geometry (metres) ---
    d_conductor: float = 0.0535  # conductor diameter
    d_insulation: float = 0.1035 # over insulation
    d_sheath: float = 0.1105     # over metallic sheath
    d_external: float = 0.1265   # over outer jacket

    # --- Insulation (XLPE) ---
    eps_r: float = 2.5           # relative permittivity
    tan_delta: float = 0.001     # dielectric loss factor
    rho_T1: float = 3.5          # K*m/W, thermal resistivity of XLPE
    rho_T3: float = 3.5          # K*m/W, thermal resistivity of jacket

    # --- Sheath ---
    R_sheath: float = 5.0e-5     # ohm/m, sheath resistance at op. temp
    lambda1: float = 0.10        # sheath loss factor (bonded system)
    lambda2: float = 0.0         # armour loss factor (none for this cable)

    # --- Installation ---
    burial_depth_m: float = 1.2
    rho_soil: float = 1.0        # K*m/W, soil thermal resistivity (moist)
    n_cores: int = 1             # single-core cable

    # --- Temperature limits (from the reference infographic) ---
    T_max_xlpe: float = 90.0     # C, max continuous conductor temp
    T_alarm: float = 70.0        # C, station alarm setpoint
    T_short_circuit: float = 250.0  # C, short-circuit conductor limit


CABLE = CableParameters()


# =====================================================================
# 2. LOSS CALCULATIONS (IEC 60287-1-1)
# =====================================================================

def conductor_dc_resistance(cable: CableParameters, theta_c: float) -> float:
    """
    R' (ohm/m): DC resistance of the conductor at operating temperature.
    IEC 60287-1-1 clause 2.1.
    """
    area_m2 = cable.conductor_area_mm2 * 1e-6
    R_dc20 = cable.rho20_copper / area_m2
    return R_dc20 * (1 + cable.alpha20_copper * (theta_c - 20.0))


def conductor_ac_resistance(cable: CableParameters, theta_c: float) -> float:
    """
    R (ohm/m): AC resistance including skin and proximity effects.
    IEC 60287-1-1 clause 2.1:  R = R_dc * (1 + ys + yp)
    """
    R_dc = conductor_dc_resistance(cable, theta_c)

    # Skin effect factor ys
    xs2 = (8 * np.pi * cable.frequency / R_dc) * 1e-7 * cable.ks
    ys = xs2**2 / (192 + 0.8 * xs2**2)

    # Proximity effect factor yp (trefoil, single-core)
    xp2 = (8 * np.pi * cable.frequency / R_dc) * 1e-7 * cable.kp
    s = cable.d_external  # axial spacing in trefoil = outer diameter
    dc_over_s = cable.d_conductor / s
    base = xp2**2 / (192 + 0.8 * xp2**2)
    yp = base * (dc_over_s**2) * (
        0.312 * dc_over_s**2 + 1.18 / (base + 0.27)
    )

    return R_dc * (1 + ys + yp)


def dielectric_loss(cable: CableParameters) -> float:
    """
    Wd (W/m): dielectric loss in the insulation.
    IEC 60287-1-1 clause 2.2:  Wd = omega * C * U0^2 * tan(delta)
    Independent of load current, so it is present even at no load.
    """
    omega = 2 * np.pi * cable.frequency
    # Capacitance per unit length (F/m) for a coaxial geometry
    C = (cable.eps_r * 1e-9) / (18 * np.log(cable.d_insulation / cable.d_conductor))
    return omega * C * cable.U0**2 * cable.tan_delta


def conductor_loss(cable: CableParameters, current_a: float, theta_c: float) -> float:
    """Wc (W/m): joule loss in the conductor, I^2 * R."""
    return current_a**2 * conductor_ac_resistance(cable, theta_c)


# =====================================================================
# 3. THERMAL RESISTANCES (IEC 60287-2-1)
# =====================================================================

def T1_insulation(cable: CableParameters) -> float:
    """
    T1 (K*m/W): thermal resistance between conductor and sheath.
    IEC 60287-2-1 clause 2.1.1.1 (single-core cable).
    """
    return (cable.rho_T1 / (2 * np.pi)) * np.log(
        cable.d_insulation / cable.d_conductor
    )


def T2_bedding() -> float:
    """
    T2 (K*m/W): between sheath and armour. Zero for an unarmoured
    single-core cable, included for completeness of the IEC chain.
    """
    return 0.0


def T3_jacket(cable: CableParameters) -> float:
    """
    T3 (K*m/W): thermal resistance of the outer jacket.
    IEC 60287-2-1 clause 2.1.3.
    """
    return (cable.rho_T3 / (2 * np.pi)) * np.log(
        cable.d_external / cable.d_sheath
    )


def T4_soil(cable: CableParameters, rho_soil: float | None = None) -> float:
    """
    T4 (K*m/W): thermal resistance of the surrounding soil for a
    directly buried cable. IEC 60287-2-1 clause 2.2.3:

        T4 = (rho_soil / 2*pi) * ln(2u + sqrt(4u^2 - 1))

    where u = 2L / De, L = burial depth, De = external diameter.

    This term dominates the total thermal resistance and is where
    hotspots come from: local soil drying raises rho_soil sharply.
    """
    rho = cable.rho_soil if rho_soil is None else rho_soil
    u = 2 * cable.burial_depth_m / cable.d_external
    return (rho / (2 * np.pi)) * np.log(2 * u + np.sqrt(4 * u**2 - 1))


# =====================================================================
# 4. STEADY-STATE CONDUCTOR TEMPERATURE
# =====================================================================

def conductor_temperature(cable: CableParameters,
                           current_a: float,
                           ambient_c: float,
                           rho_soil: float | None = None,
                           tol: float = 0.01,
                           max_iter: int = 60) -> float:
    """
    Solve for steady-state conductor temperature.

    Iterative because conductor resistance depends on temperature,
    and temperature depends on resistance. Converges in a few loops.

    dtheta = (Wc + 0.5*Wd)*T1
           + (Wc*(1+l1) + Wd)*n*T2
           + (Wc*(1+l1+l2) + Wd)*n*(T3 + T4)
    """
    T1 = T1_insulation(cable)
    T2 = T2_bedding()
    T3 = T3_jacket(cable)
    T4 = T4_soil(cable, rho_soil)

    Wd = dielectric_loss(cable)
    n = cable.n_cores
    l1, l2 = cable.lambda1, cable.lambda2

    theta_c = ambient_c + 30.0  # initial guess

    for _ in range(max_iter):
        Wc = conductor_loss(cable, current_a, theta_c)
        dtheta = (
            (Wc + 0.5 * Wd) * T1
            + (Wc * (1 + l1) + Wd) * n * T2
            + (Wc * (1 + l1 + l2) + Wd) * n * (T3 + T4)
        )
        theta_new = ambient_c + dtheta
        if abs(theta_new - theta_c) < tol:
            return theta_new
        theta_c = theta_new

    return theta_c


def sheath_temperature(cable: CableParameters,
                        theta_c: float,
                        current_a: float,
                        ambient_c: float,
                        rho_soil: float | None = None) -> float:
    """
    Outer sheath (jacket surface) temperature.

    The sheath sits after T1 in the thermal chain, so its rise above
    ambient is the total rise minus the drop across the insulation.
    This is what a surface-mounted DTS fiber actually measures, which
    is why the reference infographic lists sheath temperature
    separately from conductor temperature.
    """
    T1 = T1_insulation(cable)
    Wd = dielectric_loss(cable)
    Wc = conductor_loss(cable, current_a, theta_c)
    drop_across_insulation = (Wc + 0.5 * Wd) * T1
    return theta_c - drop_across_insulation


# =====================================================================
# 5. ROUTE PROFILE - where hotspots come from
# =====================================================================

ROUTE_LENGTH_KM = 120.0
JOINT_LOCATIONS_KM = [20.0, 40.0, 60.0, 80.0, 100.0]


@dataclass
class RouteConditions:
    """
    Physical conditions that vary ALONG the route. This is the whole
    reason a DTS system exists: a single rating calculation assumes
    uniform conditions, but real routes are not uniform.
    """
    ambient_c: float = 25.0
    soil_rho_base: float = 1.0        # K*m/W, normal moist soil
    hotspot_km: float | None = 42.3
    hotspot_soil_rho: float = 1.8     # K*m/W, dried-out soil at hotspot
    hotspot_width_km: float = 4.0
    joint_soil_rho_bump: float = 0.25 # extra resistivity near joints
    joint_width_km: float = 1.5
    joints: list = field(default_factory=lambda: list(JOINT_LOCATIONS_KM))


def soil_resistivity_profile(distance_km: np.ndarray,
                              cond: RouteConditions) -> np.ndarray:
    """
    Soil thermal resistivity as a function of position along the route.

    Physically justified: thermal resistivity rises where soil dries
    out (moisture migration away from a hot cable), where backfill was
    poorly compacted, or near joint bays where the excavation was
    refilled. Elevated rho_soil raises T4, which raises conductor
    temperature. That is the mechanism behind a real hotspot.
    """
    rho = np.full_like(distance_km, cond.soil_rho_base)

    for j in cond.joints:
        rho += cond.joint_soil_rho_bump * np.exp(
            -((distance_km - j) ** 2) / (2 * cond.joint_width_km ** 2)
        )

    if cond.hotspot_km is not None:
        excess = cond.hotspot_soil_rho - cond.soil_rho_base
        rho += excess * np.exp(
            -((distance_km - cond.hotspot_km) ** 2)
            / (2 * cond.hotspot_width_km ** 2)
        )

    return rho


def generate_profile(load_factor: float = 1.0,
                      cond: RouteConditions | None = None,
                      cable: CableParameters = CABLE,
                      n_points: int = 300,
                      noise_std_c: float = 0.3,
                      seed: int | None = 1):
    """
    Compute conductor and sheath temperature at every point along the
    route by solving the IEC 60287 equations independently at each
    position with that position's local soil resistivity.

    Returns (distance_km, conductor_temp_c, sheath_temp_c, soil_rho).
    """
    if cond is None:
        cond = RouteConditions()

    rng = np.random.default_rng(seed)
    distance = np.linspace(0.0, ROUTE_LENGTH_KM, n_points)
    rho_profile = soil_resistivity_profile(distance, cond)
    current = load_factor * cable.rated_current

    t_cond = np.empty_like(distance)
    t_sheath = np.empty_like(distance)

    for i, rho in enumerate(rho_profile):
        tc = conductor_temperature(cable, current, cond.ambient_c, rho_soil=rho)
        t_cond[i] = tc
        t_sheath[i] = sheath_temperature(cable, tc, current,
                                          cond.ambient_c, rho_soil=rho)

    # Measurement noise. Real DTS units quote roughly +/- 0.5 K.
    t_cond += rng.normal(0, noise_std_c, size=distance.shape)
    t_sheath += rng.normal(0, noise_std_c, size=distance.shape)

    return distance, t_cond, t_sheath, rho_profile


# =====================================================================
# 6. HOTSPOT AND ALARM EVALUATION
# =====================================================================

def find_hotspot(distance_km, conductor_temp_c):
    """Return (location_km, peak_temp_c) of the hottest point."""
    idx = int(np.argmax(conductor_temp_c))
    return float(distance_km[idx]), float(conductor_temp_c[idx])


def check_alarm(conductor_temp_c, threshold_c: float = CABLE.T_alarm) -> bool:
    """True if any point on the route reaches the alarm setpoint."""
    return bool(np.max(conductor_temp_c) >= threshold_c)


def rating_margin(conductor_temp_c, cable: CableParameters = CABLE) -> float:
    """Kelvin of headroom remaining before the 90 C XLPE limit."""
    return cable.T_max_xlpe - float(np.max(conductor_temp_c))


# =====================================================================
# 7. SELF-CHECK
# =====================================================================

if __name__ == "__main__":
    print("=" * 62)
    print("IEC 60287 model - component values")
    print("=" * 62)
    print(f"  T1 (insulation)  = {T1_insulation(CABLE):.4f} K.m/W")
    print(f"  T3 (jacket)      = {T3_jacket(CABLE):.4f} K.m/W")
    print(f"  T4 (soil, 1.0)   = {T4_soil(CABLE, 1.0):.4f} K.m/W")
    print(f"  T4 (soil, 2.8)   = {T4_soil(CABLE, 2.8):.4f} K.m/W  <- hotspot")
    print(f"  Wd (dielectric)  = {dielectric_loss(CABLE):.3f} W/m")
    R = conductor_ac_resistance(CABLE, 70.0)
    print(f"  R_ac at 70 C     = {R*1e6:.3f} micro-ohm/m")
    print(f"  Wc at rated I    = {conductor_loss(CABLE, CABLE.rated_current, 70.0):.2f} W/m")

    print()
    print("=" * 62)
    print("Route profile at rated load")
    print("=" * 62)
    d, tc, ts, rho = generate_profile(load_factor=1.0)
    loc, peak = find_hotspot(d, tc)
    print(f"  Normal section conductor temp : {tc[5]:6.1f} C")
    print(f"  Normal section sheath temp    : {ts[5]:6.1f} C")
    print(f"  Hotspot location              : {loc:6.1f} km")
    print(f"  Hotspot conductor temp        : {peak:6.1f} C")
    print(f"  Alarm (>= {CABLE.T_alarm:.0f} C)             : "
          f"{'ACTIVE' if check_alarm(tc) else 'clear'}")
    print(f"  Margin to 90 C XLPE limit     : {rating_margin(tc):6.1f} K")
