"""
DTS to PLC Modbus Bridge
=========================
Connects the simulated DTS interrogator (thermal_model.py) to the
OpenPLC runtime over Modbus TCP.

Role in the system:

    thermal_model.py  ->  THIS BRIDGE  ->  OpenPLC  ->  dashboard.py
      (DTS sensing)       (Modbus TCP)    (RTU logic)    (HMI/SCADA)

The bridge writes the hottest measured conductor temperature into the
PLC, then reads back the alarm outputs the PLC has decided on. The
alarm decision is made by LADDER LOGIC inside the PLC, not in Python.
That is the point of the exercise: the protection logic lives where it
would in a real installation.


ADDRESS MAP  (must match the ladder program exactly)
----------------------------------------------------
Written BY this bridge INTO the PLC:

    IEC address   Modbus block        Modbus addr   Meaning
    %QW0          Holding register    0             peak temp, deg C x 10

Read FROM the PLC by this bridge:

    %QX0.0        Coil                0             beacon
    %QX0.1        Coil                1             buzzer
    %QX0.2        Coil                2             limit_exceeded
    %QX0.3        Coil                3             comms_ok
    %QW10         Holding register    10            alarm_state (0-4)

WHY HOLDING REGISTERS AND NOT %IW
----------------------------------
In OpenPLC, %IW maps to Modbus INPUT REGISTERS and %IX maps to DISCRETE
INPUTS. Both are READ-ONLY to a Modbus master. In a real installation
the DTS interrogator would be wired to physical input terminals, so %IW
would be correct. Here the interrogator is simulated by this script,
which reaches the PLC over the network as a Modbus MASTER, so it must
write into a writable block: holding registers (%QW). Worth one sentence
in the report.

Coil bit maths: %QX<byte>.<bit> is Modbus coil (byte * 8 + bit).
So %QX0.0 is coil 0 and %QX0.3 is coil 3. An earlier version of this
file used %QX100.0, which is coil 800, not coil 100.


USAGE
-----
  1. Start the OpenPLC Runtime and upload the ladder program from the
     OpenPLC Editor (Build and upload), then start the PLC.
  2. python plc_bridge.py

If OpenPLC is not running, the bridge falls back to a local simulation
of the same logic so the rest of the system keeps working. That
fallback is clearly flagged, and should be disclosed if you demo
without the PLC running.
"""

import time
import logging
from dataclasses import dataclass

import numpy as np

from thermal_model import (
    CABLE, RouteConditions, generate_profile,
    find_hotspot, check_alarm, rating_margin,
)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s  %(levelname)-7s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("bridge")

# --- Modbus configuration -------------------------------------------
PLC_HOST = "127.0.0.1"   # OpenPLC runtime on this laptop
PLC_PORT = 502           # standard Modbus TCP port
UNIT_ID = 1

# --- Register map, matching the ladder program ----------------------
# Inputs to the PLC (holding registers, writable by this master)
REG_PEAK_TEMP = 0        # %QW0,  deg C x 10

# Outputs from the PLC
COIL_BEACON = 0          # %QX0.0
COIL_BUZZER = 1          # %QX0.1
COIL_LIMIT = 2           # %QX0.2
COIL_COMMS = 3           # %QX0.3
REG_ALARM_STATE = 10     # %QW10

# How long a single evaluation will wait for the 3 s on-delay in rung 4
# to complete before giving up and reporting whatever the PLC says. Set a
# little above the timer preset so a normal alarm always has time to show.
ON_DELAY_WAIT_S = 4.0

# Rungs 6 and 7 (comms watchdog, composite state word) are optional
# extras. Until they are built, comms_ok and alarm_state simply read
# back as 0 from the PLC. Set this False to stop trusting them and
# derive the display state from the beacon/limit coils instead.
PLC_DRIVES_STATE_WORD = False


@dataclass
class PlcStatus:
    """What the PLC reports back after evaluating its logic."""
    beacon: bool = False
    buzzer: bool = False
    limit_exceeded: bool = False
    comms_ok: bool = True
    alarm_state: int = 0
    source: str = "plc"     # "plc" or "fallback"

    @property
    def state_text(self) -> str:
        return {
            0: "Normal",
            1: "Warning",
            2: "Alarm",
            3: "Critical",
            4: "Comms failure",
        }.get(self.alarm_state, "Unknown")


def _call(fn, *args, **kwargs):
    """
    pymodbus renamed the unit-id keyword between versions (slave= in
    3.x, device_id= in the newest releases). Try both so the script
    runs whichever version pip installed.
    """
    try:
        return fn(*args, slave=UNIT_ID, **kwargs)
    except TypeError:
        return fn(*args, device_id=UNIT_ID, **kwargs)


class PlcLink:
    """
    Thin wrapper over a Modbus TCP connection to the OpenPLC runtime.

    Degrades gracefully: if pymodbus is missing or the PLC is not
    reachable, is_connected stays False and the caller can use the
    fallback logic instead of crashing mid-demo.
    """

    def __init__(self, host: str = PLC_HOST, port: int = PLC_PORT):
        self.host = host
        self.port = port
        self.client = None
        self.is_connected = False

    def connect(self) -> bool:
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError:
            log.warning("pymodbus not installed. Run: pip install pymodbus")
            return False

        try:
            self.client = ModbusTcpClient(self.host, port=self.port, timeout=2)
            self.is_connected = self.client.connect()
            if self.is_connected:
                log.info("Connected to OpenPLC at %s:%d", self.host, self.port)
            else:
                log.warning("OpenPLC not reachable at %s:%d", self.host, self.port)
            return self.is_connected
        except Exception as exc:
            log.warning("Modbus connection failed: %s", exc)
            self.is_connected = False
            return False

    def write_measurements(self, peak_temp_c: float) -> bool:
        """
        Push the DTS reading into the PLC's holding register %QW0.

        Scaled by 10 because Modbus registers carry 16-bit integers,
        not floats: 72.4 C travels as the integer 724, and the ladder
        divides by 10.0 in rung 1. Standard SCADA practice.
        """
        if not self.is_connected:
            return False
        try:
            value = int(round(peak_temp_c * 10))
            # 16-bit signed range, so clamp rather than wrap around
            value = max(-32768, min(32767, value))
            result = _call(self.client.write_register, REG_PEAK_TEMP, value)
            return not result.isError()
        except Exception as exc:
            log.error("Write failed: %s", exc)
            self.is_connected = False
            return False

    def read_status(self) -> "PlcStatus | None":
        """Read back the alarm decisions the PLC has made."""
        if not self.is_connected:
            return None
        try:
            coils = _call(self.client.read_coils, COIL_BEACON, count=4)
            regs = _call(self.client.read_holding_registers,
                         REG_ALARM_STATE, count=1)
            if coils.isError() or regs.isError():
                return None

            beacon = bool(coils.bits[COIL_BEACON])
            buzzer = bool(coils.bits[COIL_BUZZER])
            limit = bool(coils.bits[COIL_LIMIT])
            comms = bool(coils.bits[COIL_COMMS])
            state = int(regs.registers[0])

            if not PLC_DRIVES_STATE_WORD:
                # Rungs 6 and 7 not built yet: derive the display state
                # from the coils the ladder actually drives.
                comms = True
                state = 3 if limit else (2 if beacon else 0)

            return PlcStatus(
                beacon=beacon,
                buzzer=buzzer,
                limit_exceeded=limit,
                comms_ok=comms,
                alarm_state=state,
                source="plc",
            )
        except Exception as exc:
            log.error("Read failed: %s", exc)
            self.is_connected = False
            return None

    def close(self):
        if self.client:
            self.client.close()
        self.is_connected = False


def fallback_logic(peak_temp_c: float) -> PlcStatus:
    """
    Mirror of the PLC logic, evaluated in Python.

    Used only when the OpenPLC runtime is unavailable, so that the
    dashboard still works. The thresholds and state codes match the
    ladder program exactly. Note this version has no on-delay timer
    and no hysteresis latch, since those need a real scan cycle.
    """
    if peak_temp_c >= CABLE.T_max_xlpe:
        state = 3
    elif peak_temp_c >= CABLE.T_alarm:
        state = 2
    elif peak_temp_c >= 60.0:
        state = 1
    else:
        state = 0

    alarming = peak_temp_c >= CABLE.T_alarm
    return PlcStatus(
        beacon=alarming,
        buzzer=alarming,
        limit_exceeded=peak_temp_c >= CABLE.T_max_xlpe,
        comms_ok=True,
        alarm_state=state,
        source="fallback",
    )


def evaluate_once(load_factor: float = 1.0,
                  cond: "RouteConditions | None" = None,
                  link: "PlcLink | None" = None,
                  wait_for_timer: bool = True):
    """
    Run one full measurement cycle:
      1. Simulate the DTS temperature profile along the route
      2. Extract the hotspot
      3. Send the peak temperature to the PLC
      4. Read back the PLC's alarm decision

    Returns (distance, t_cond, t_sheath, rho, hotspot_km, peak_c, status).
    """
    if cond is None:
        cond = RouteConditions()

    distance, t_cond, t_sheath, rho = generate_profile(
        load_factor=load_factor, cond=cond
    )
    hotspot_km, peak_c = find_hotspot(distance, t_cond)

    status = None
    if link is not None and link.is_connected:
        if link.write_measurements(peak_c):
            # One scan cycle is enough for the comparison rungs, but the
            # beacon sits behind a 3 second on-delay in rung 4. A single
            # snapshot taken 0.15 s after writing would report the beacon
            # still off while the temperature is clearly in alarm, which
            # looks like a fault and is really just the timer running.
            #
            # So: read immediately, and if the temperature is above the
            # setpoint but the beacon has not asserted yet, keep polling
            # until the timer completes or the wait budget runs out.
            time.sleep(0.15)
            status = link.read_status()

            if wait_for_timer and status is not None:
                deadline = time.time() + ON_DELAY_WAIT_S
                while (peak_c >= CABLE.T_alarm and not status.beacon
                       and time.time() < deadline):
                    time.sleep(0.25)
                    newer = link.read_status()
                    if newer is None:
                        break
                    status = newer

    if status is None:
        status = fallback_logic(peak_c)

    return distance, t_cond, t_sheath, rho, hotspot_km, peak_c, status


if __name__ == "__main__":
    link = PlcLink()
    link.connect()

    log.info("Starting continuous monitoring. Ctrl+C to stop.")
    log.info("-" * 58)

    try:
        cycle = 0
        while True:
            # Slowly ramp load so the alarm threshold is crossed live.
            # Useful for the demo: let it run and watch the alarm trip
            # three seconds after the temperature passes 70 C.
            load = 0.70 + 0.30 * min(cycle / 25.0, 1.0)

            d, tc, ts, rho, hs_km, peak, status = evaluate_once(
                load_factor=load, link=link, wait_for_timer=False
            )
            margin = rating_margin(tc)

            log.info(
                "load %3d%%  hotspot %5.1f km  peak %5.1f C  "
                "margin %5.1f K  state %-13s [%s]",
                int(load * 100), hs_km, peak, margin,
                status.state_text, status.source,
            )

            if status.beacon:
                log.warning("  >>> BEACON ON at %.1f km (%.1f C)", hs_km, peak)
            if status.limit_exceeded:
                log.error("  >>> XLPE 90 C LIMIT EXCEEDED")

            cycle += 1
            time.sleep(2)

    except KeyboardInterrupt:
        log.info("Stopped.")
    finally:
        link.close()