#!/usr/bin/env python3
"""
Unit test: catapult launch integrator windup fix.

Simulates the pitch rate PID integrator behaviour during:
  1. Pre-launch standby  (turbine idling, plane constrained on rail)
  2. Catapult launch     (high AccX spike)
  3. First second of flight

Runs two scenarios side-by-side:
  STOCK   - original ArduPlane (throttle-gated pre-takeoff reset, no launch detection)
  PATCHED - our fix           (airspeed-gated reset + catapult AccX detection)

Parameters taken directly from GR-005_2_Crush.BIN:
  PTCH_RATE_I     = 0.15
  PTCH_RATE_IMAX  = 0.666   (radians domain; output scaled by degrees(1) = 57.296)
  TKOFF_THR_MINACC = 30.0   m/s^2
  Loop rate        = 400 Hz
"""

import math

# ── Parameters (from log) ─────────────────────────────────────────────────────
KI          = 0.15          # PTCH_RATE_I
IMAX        = 0.666         # PTCH_RATE_IMAX  (radians domain)
DEG_SCALE   = math.degrees(1)   # 57.296  — converts raw PID to degrees of servo
IMAX_DEG    = IMAX * DEG_SCALE  # 38.13°  — saturation limit in output units
DT          = 1.0 / 400     # 400 Hz loop
MINACC      = 30.0          # TKOFF_THR_MINACC  m/s^2
INHIBIT_MS  = 300           # integrator suppression window after launch

# ── Scenario timeline ─────────────────────────────────────────────────────────
#
#  0 .. STANDBY_S   : plane on rail, turbine idling
#                     actual pitch = 12°, desired pitch = ~2-5°
#                     pitch rate error ≈ -0.6 deg/s  (from log PIDP.Err)
#                     throttle input = IDLE (non-zero)
#                     airspeed = 0 m/s
#
#  STANDBY_S        : catapult fires  →  AccX spikes to 45 → 129 m/s^2
#
#  STANDBY_S .. end : free flight, integrators accumulate normally

STANDBY_S   = 5.0           # seconds on the rail (shortened from real ~100s for speed)
FLIGHT_S    = 1.5           # seconds of post-launch data to show
TOTAL_STEPS = int((STANDBY_S + FLIGHT_S) / DT)

# Typical pitch rate error on the rail (from PIDP.Err in log): -0.6 deg/s
# After launch the error swings large due to the IMU disturbance then settles
RAIL_RATE_ERR = -0.6        # deg/s  (constant while constrained)

def accel_x_at(t):
    """Body-frame forward acceleration profile from the log."""
    dt = t - STANDBY_S
    if dt < 0:
        return 2.0          # gravity component along body X at 12° pitch (9.81*sin12°)
    if dt < 0.02:
        return 45.0         # first sample after launch
    if dt < 0.10:
        return 129.0        # peak catapult pulse
    if dt < 0.25:
        return 12.0         # residual
    return 4.5              # climbing thrust

def pitch_rate_error_at(t):
    """Commanded vs actual pitch rate error (deg/s) from PIDP.Err in log."""
    dt = t - STANDBY_S
    if dt < 0:
        return RAIL_RATE_ERR
    if dt < 0.15:
        return -5.0         # large error during IMU saturation
    if dt < 0.40:
        return 0.8          # controller catching up
    return 0.5              # normal climb

def ff_at(t):
    """Feed-forward contribution to servo output (degrees)."""
    dt = t - STANDBY_S
    if dt < 0:
        return -11.0        # standing on rail (from log PIDP.FF)
    if dt < 0.15:
        return -17.0        # launch spike
    return -14.0            # cruise climb

# ── PID integrator model ───────────────────────────────────────────────────────

def run_scenario(name, airspeed_reset, launch_reset):
    """
    airspeed_reset : bool  — fix #2 (replace throttle gate with airspeed gate)
    launch_reset   : bool  — fix #1 (reset integrators on AccX > MINACC)
    """
    I_raw   = 0.0       # integrator in radians domain
    launch_ms = None    # timestamp of catapult detection (milliseconds)
    I_at_launch = None  # captured the moment AccX first exceeds threshold
    servo_at_launch = None

    print(f"\n{'='*72}")
    print(f"  {name}")
    print(f"{'='*72}")
    print(f"  {'Time(s)':>8}  {'AccX':>7}  {'RateErr':>8}  {'I_raw':>9}  "
          f"{'I_out°':>8}  {'FF°':>7}  {'Total°':>8}  Note")
    print(f"  {'-'*8}  {'-'*7}  {'-'*8}  {'-'*9}  {'-'*8}  {'-'*7}  {'-'*8}  ----")

    prev_t_ms = 0
    sample_every = max(1, int(0.08 / DT))   # print every ~80 ms

    for step in range(TOTAL_STEPS):
        t        = step * DT
        t_ms     = int(t * 1000)
        ax       = accel_x_at(t)
        rate_err = pitch_rate_error_at(t)
        ff       = ff_at(t)
        airspeed = 0.0 if t < STANDBY_S else (t - STANDBY_S) * 15.0  # ramps up after launch

        # ── Accumulate integrator ──────────────────────────────────────────
        # Mirrors AC_PID_Basic::update_i():  I += (err * ki) * dt
        I_raw += (rate_err * DEG_SCALE * KI) * DT   # error scaled same as in AP_FW_Controller
        # Anti-windup clamp (IMAX in output-degree units → convert back)
        I_raw_deg = I_raw * DEG_SCALE
        if abs(I_raw_deg) > IMAX_DEG:
            I_raw = math.copysign(IMAX_DEG / DEG_SCALE, I_raw)

        # ── Fix #2: airspeed-gated pre-takeoff reset ───────────────────────
        if airspeed_reset:
            # Reset when not moving through air, low altitude, not climbing
            # (replaces the original throttle-gated condition)
            if airspeed < 2.0:   # airspeed_EAS < 2 m/s
                I_raw = 0.0

        # ── Fix #1: catapult launch detection ─────────────────────────────
        if launch_reset:
            if ax > MINACC and launch_ms is None:
                launch_ms = t_ms
            if launch_ms is not None:
                since = t_ms - launch_ms
                if since < INHIBIT_MS:
                    I_raw = 0.0
                elif since > 5000:
                    launch_ms = None   # ready for next launch

        # ── Compute servo output contribution ─────────────────────────────
        I_out   = I_raw * DEG_SCALE
        P_out   = -1.5                  # approximate P term (from log)
        total   = ff + P_out + I_out
        # Servo is clamped to ±45°
        servo   = max(-45.0, min(45.0, total))

        # ── Capture state at launch ───────────────────────────────────────
        if I_at_launch is None and ax > MINACC:
            I_at_launch = I_raw * DEG_SCALE
            servo_at_launch = max(-45.0, min(45.0, ff + (-1.5) + I_at_launch))

        # ── Print sample row ───────────────────────────────────────────────
        if step % sample_every == 0:
            note = ""
            if abs(t - STANDBY_S) < DT * 2:
                note = "← LAUNCH"
            elif I_out < -35:
                note = "WINDUP"
            elif servo <= -44.9:
                note = "FULL NOSE-DOWN"
            elif t > STANDBY_S and I_out > -5:
                note = "clean"
            print(f"  {t:>8.3f}  {ax:>7.1f}  {rate_err:>8.3f}  "
                  f"{I_raw:>9.5f}  {I_out:>8.3f}  {ff:>7.1f}  {servo:>8.2f}  {note}")

    # ── Summary ───────────────────────────────────────────────────────────
    if I_at_launch is None:
        I_at_launch = I_raw * DEG_SCALE
    if servo_at_launch is None:
        servo_at_launch = max(-45.0, min(45.0, ff_at(STANDBY_S) + (-1.5) + I_at_launch))
    verdict = "FULL NOSE-DOWN ← CRASH" if servo_at_launch <= -44.9 else "controlled ✓"
    print(f"\n  RESULT: I_out at launch = {I_at_launch:+.3f}°   "
          f"Elevator at launch = {servo_at_launch:+.2f}°   [{verdict}]")


# ── Run both scenarios ─────────────────────────────────────────────────────────

print("\n" + "="*72)
print("  CATAPULT LAUNCH INTEGRATOR WINDUP — UNIT TEST")
print(f"  Parameters: KI={KI}, IMAX={IMAX} ({IMAX_DEG:.2f}°), "
      f"MINACC={MINACC} m/s², loop={int(1/DT)}Hz")
print(f"  Standby: {STANDBY_S}s at RAIL_RATE_ERR={RAIL_RATE_ERR}°/s  "
      f"(throttle non-zero, airspeed=0)")
print("="*72)

run_scenario(
    "STOCK   — original ArduPlane (throttle-gated reset, no launch detection)",
    airspeed_reset=False,
    launch_reset=False,
)

run_scenario(
    "PATCHED — fix #2 only: airspeed-gated pre-takeoff reset",
    airspeed_reset=True,
    launch_reset=False,
)

run_scenario(
    "PATCHED — fix #1 only: catapult AccX detection + 300ms suppression",
    airspeed_reset=False,
    launch_reset=True,
)

run_scenario(
    "PATCHED — both fixes combined (as committed)",
    airspeed_reset=True,
    launch_reset=True,
)

print("\n" + "="*72)
print("  PASS criteria:")
print("  - Stock elevator at launch <= -44.9° (full nose-down) → confirms bug")
print("  - Each patched variant elevator > -30°              → confirms fix")
print("="*72 + "\n")
