(*
   DTS CABLE TEMPERATURE MONITORING - RTU ALARM LOGIC
   ===================================================
   REFERENCE ONLY. The program actually running is the LADDER DIAGRAM
   built in the OpenPLC Editor. This Structured Text file documents the
   same logic in text form, which is useful as a report appendix and as
   a check that the ladder rungs do what they are meant to.

   Do not upload this file. Upload the ladder project.

   MODBUS ADDRESS MAP (matches plc_bridge.py)
   ------------------------------------------
   Written BY the DTS simulator (Python) INTO the PLC:
     %QW0     peak_temp_x10   Hottest conductor temp, deg C x 10

   Read FROM the PLC by the bridge:
     %QX0.0   beacon          Local alarm beacon
     %QX0.1   buzzer          Local audible alarm        (not yet built)
     %QX0.2   limit_exceeded  XLPE 90 C limit exceeded
     %QX0.3   comms_ok        Communication healthy      (not yet built)
     %QW10    alarm_state     0 normal .. 4 comms fail   (not yet built)

   Incoming measurements use %QW (holding registers) rather than %IW
   (input registers) because a Modbus master cannot write to input
   registers. In a real installation the interrogator would be wired to
   physical input terminals and %IW would be correct.

   Temperatures travel as integers scaled by 10 because Modbus holding
   registers carry 16-bit integers, not floats. Standard SCADA practice.
*)

PROGRAM dts_alarm
  VAR
    (* --- Input from the DTS interrogator --- *)
    peak_temp_x10   AT %QW0   : INT;

    (* --- Outputs to local alarm devices and SCADA --- *)
    beacon          AT %QX0.0 : BOOL;
    buzzer          AT %QX0.1 : BOOL;
    limit_exceeded  AT %QX0.2 : BOOL;
    comms_ok        AT %QX0.3 : BOOL;
    alarm_state     AT %QW10  : INT;

    (* --- Internal working variables --- *)
    temp_real       : REAL;   (* between TO_REAL and DIV in rung 1 *)
    peak_temp       : REAL;
    alarm_active    : BOOL := FALSE;
    value_changed   : BOOL;
    last_value      : INT := 0;
    alarm_delay     : TON;
    watchdog        : TON;
  END_VAR

  (* === RUNG 1 - rescale to engineering units ======================
     TO_REAL then DIV by 10.0. An INT cannot be divided by a REAL
     directly, hence the conversion step and the intermediate
     temp_real variable.                                            *)
  temp_real := TO_REAL(peak_temp_x10);
  peak_temp := temp_real / 10.0;

  (* === RUNGS 2 and 3 - alarm latch with hysteresis ================
     Asserts at 70.0 C, clears only once the temperature falls back
     to 68.0 C. The 2 K deadband stops the beacon chattering when
     the temperature hovers exactly on the setpoint.
     In ladder: GE -> (S) coil, and LT -> (R) coil, on two rungs.   *)
  IF peak_temp >= 70.0 THEN
    alarm_active := TRUE;
  ELSIF peak_temp < 68.0 THEN
    alarm_active := FALSE;
  END_IF;

  (* === RUNG 4 - on-delay, then beacon =============================
     3 seconds rejects transient measurement spikes. Real DTS
     interrogators produce occasional noisy traces, and an operator
     should not be called out for a single bad sample.              *)
  alarm_delay(IN := alarm_active, PT := T#3s);
  beacon := alarm_delay.Q;

  (* === RUNG 5 - hard limit ========================================
     90 C is the XLPE continuous conductor limit. Reported with no
     delay and no deadband: it is a safety limit, not an alarm.     *)
  limit_exceeded := (peak_temp >= 90.0);

  (* === RUNG 6 (OPTIONAL, NOT YET BUILT) - comms watchdog ==========
     If the incoming value stops changing for 10 seconds, the link to
     the DTS unit is presumed lost. A real RTU must distinguish
     "cable is cool" from "I am not receiving data".                *)
  (*
  value_changed := (peak_temp_x10 <> last_value);
  last_value    := peak_temp_x10;
  watchdog(IN := NOT value_changed, PT := T#10s);
  comms_ok := NOT watchdog.Q;
  *)

  (* === RUNG 7 (OPTIONAL, NOT YET BUILT) - state word for SCADA ====
     In ladder this needs MOVE blocks ordered so that the higher
     severity states override the lower ones.                       *)
  (*
  IF NOT comms_ok THEN
    alarm_state := 4;
  ELSIF peak_temp >= 90.0 THEN
    alarm_state := 3;
  ELSIF alarm_delay.Q THEN
    alarm_state := 2;
  ELSIF peak_temp >= 60.0 THEN
    alarm_state := 1;
  ELSE
    alarm_state := 0;
  END_IF;
  *)

END_PROGRAM
