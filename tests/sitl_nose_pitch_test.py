#!/usr/bin/env python3
"""
SITL test: no nose-pitch-down after catapult launch.

Verifies that pitch integrator windup during rail standby does NOT cause
the plane to "клюнути носом" (pitch-down dive) once the catapult fires.

Three phases (all times are simulated):
  Phase 1 — Rail standby (10 s sim):
      Plane on 12.35° catapult rail, turbine idle, NOT armed.
      AHRS_TRIM_Y = -0.2155 rad → FC sees +12.35° pitch error.
      Fix #2 (airspeed-gated reset) keeps integrators ≈ 0.
      Pass: max |I| < IMAX (no saturation).

  Phase 2 — Catapult launch (5 s sim):
      AHRS_TRIM_Y reset to 0 before arm (clean SITL flight physics).
      Armed + full throttle.  TKOFF_THR_MINACC = 3 m/s² ensures Fix #1
      fires on SITL-level body-X acceleration.
      Pass: AccX > threshold  AND  |I| < 1° within 1 s after reset.

  Phase 3 — Cruise (20 s sim):
      Throttle at cruise level, FBWA holds attitude.
      Pass: pitch never drops below -5° (no dive from integrator kick).

Usage:
  python3 tests/sitl_nose_pitch_test.py [--binary path/to/arduplane]

Requires:
  build/sitl/bin/arduplane   (python3 waf configure --board sitl &&
                               python3 waf --targets bin/arduplane)
"""

import argparse, math, os, signal, subprocess, sys, time
from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_BINARY = os.path.join(
    os.path.dirname(__file__), '..', 'build', 'sitl', 'bin', 'arduplane')
WORK_DIR  = '/tmp/sitl_nose_pitch_test'
PORT      = 5775
SPEEDUP   = 5

CATAPULT_PITCH_DEG = 12.35
AHRS_TRIM_Y_RAIL   = -math.radians(CATAPULT_PITCH_DEG)   # ≈ -0.2155 rad

STANDBY_S = 10    # simulated seconds on rail before launch
LAUNCH_S  = 5     # simulated seconds of launch / climb-out
CRUISE_S  = 20    # simulated seconds of cruise

# Pass thresholds
IMAX_DEG       = math.degrees(0.666)   # ≈ 38.2° — hard integrator limit
PITCH_MIN_DEG  = -5.0                  # Phase 3: nose-down dive threshold
TKOFF_MINACC   = 3.0                   # m/s² — low so SITL throttle triggers Fix #1

PARAMS = {
    # PID matching real GR-005 log
    'PTCH_RATE_I':      0.15,
    'PTCH_RATE_IMAX':   0.666,
    'PTCH_RATE_FF':     0.345,
    # Low catapult threshold so SITL engine thrust is enough to fire Fix #1
    'TKOFF_THR_MINACC': TKOFF_MINACC,
    # Calm day — pure integrator scenario
    'SIM_WIND_SPD':     0.0,
    'SIM_WIND_DIR':     0.0,
    'SIM_ARSPD_OFS':    0.0,
    # Prevent boot-time calibration saving a ~2014 offset → ~63 m/s ghost reading
    'ARSPD_SKIP_CAL':   1,
    'ARSPD_OFFSET':     0.0,
    # Log while disarmed so Phase 1 PIDP data is captured
    'LOG_DISARMED':     1,
    # Zero out FBWA nose-down-at-idle feature — not relevant to catapult scenario
    'STAB_PITCH_DOWN':  0,
    # Rail angle: FC sees +12.35° pitch → generates nose-down pitch demand
    'AHRS_TRIM_Y':      AHRS_TRIM_Y_RAIL,
    # Cruise settings for Phase 3
    'TRIM_THROTTLE':    0.6,
    'TRIM_ARSPD_CM':    1500,   # 15 m/s cruise target
}

# ── Helpers ───────────────────────────────────────────────────────────────────

def start_sitl(binary):
    import shutil
    shutil.rmtree(WORK_DIR, ignore_errors=True)
    os.makedirs(WORK_DIR, exist_ok=True)
    cmd = [
        os.path.abspath(binary),
        '--model', 'plane',
        '--home', '51.0,0.0,100,0',   # 100 m ASL gives flight room
        f'--speedup={SPEEDUP}',
        f'--serial0=tcp:{PORT}',
    ]
    print(f"Starting SITL (speedup={SPEEDUP}x)...")
    proc = subprocess.Popen(
        cmd, cwd=WORK_DIR,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    time.sleep(5)
    return proc


def connect(port):
    for attempt in range(30):
        try:
            m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
            hb = m.wait_heartbeat(timeout=5)
            if m.target_system == 0:
                time.sleep(1)
                continue
            print(f"  Connected (sysid={m.target_system})")
            return m
        except Exception:
            time.sleep(1)
    raise RuntimeError("Failed to connect to SITL")


def set_param(m, name, value, retries=5):
    for _ in range(retries):
        m.mav.param_set_send(
            m.target_system, m.target_component,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            return
    print(f"  WARNING: no ACK for {name}={value}")


def set_mode(m, mode_name):
    mode_id = m.mode_mapping().get(mode_name)
    if mode_id is None:
        raise ValueError(f"Unknown mode {mode_name}")
    for _ in range(5):
        m.mav.set_mode_send(
            m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            return


def arm_force(m):
    for _ in range(10):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0)
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def rc_override(m, throttle_pwm=1300, elevator_pwm=1500):
    """Send RC override: ch1=aileron, ch2=elevator, ch3=throttle, ch4=rudder."""
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        1500, elevator_pwm, throttle_pwm, 1500,
        0, 0, 0, 0)


def drain(m, seconds, throttle_pwm=1300, elevator_pwm=1500):
    """Pump MAVLink messages for `seconds` real seconds while sending RC override."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        rc_override(m, throttle_pwm=throttle_pwm, elevator_pwm=elevator_pwm)
        m.recv_match(blocking=True, timeout=0.02)


def stop_sitl(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


# ── Log analysis ──────────────────────────────────────────────────────────────

def read_log(log_path):
    """Parse PIDP, ATT, IMU from BIN log.
    Returns:
      pidp — list of (TimeUS, I_deg)
      att  — list of (TimeUS, Pitch_deg)
      imu  — list of (TimeUS, AccX_ms2)
    """
    mlog = mavutil.mavlink_connection(log_path, dialect='ardupilotmega')
    pidp, att, imu = [], [], []
    while True:
        msg = mlog.recv_match(type=['PIDP', 'ATT', 'IMU'], blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == 'PIDP':
            pidp.append((msg.TimeUS, msg.I))
        elif t == 'ATT':
            att.append((msg.TimeUS, msg.Pitch))
        elif t == 'IMU':
            imu.append((msg.TimeUS, msg.AccX))
    return pidp, att, imu


def find_launch_time_us(imu, threshold):
    """Return TimeUS of first IMU sample with AccX > threshold."""
    for ts, ax in imu:
        if ax > threshold:
            return ts
    return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', default=DEFAULT_BINARY)
    args = parser.parse_args()

    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        print(f"ERROR: binary not found: {binary}")
        sys.exit(1)

    total_sim = STANDBY_S + LAUNCH_S + CRUISE_S
    print(f"\n{'='*65}")
    print("SITL nose-pitch test — does the plane dive after catapult?")
    print(f"{'='*65}")
    print(f"Binary    : {binary}")
    print(f"Rail angle: {CATAPULT_PITCH_DEG}° nose-up "
          f"(AHRS_TRIM_Y = {AHRS_TRIM_Y_RAIL:.4f} rad)")
    print(f"Phases    : {STANDBY_S}s standby + {LAUNCH_S}s launch + {CRUISE_S}s cruise (sim)")
    print(f"Wall time : ≈ {total_sim/SPEEDUP + 15:.0f}s (includes boot overhead)")
    print(f"Pass criteria:")
    print(f"  Phase 1 — max |I| < {IMAX_DEG:.1f}° (no integrator saturation on rail)")
    print(f"  Phase 2 — AccX > {TKOFF_MINACC} m/s²  AND  |I| < 1° after Fix #1 reset")
    print(f"  Phase 3 — pitch never drops below {PITCH_MIN_DEG}° (no nose-dive)")

    proc = start_sitl(binary)
    log_path = None
    try:
        m = connect(PORT)

        print(f"\n  Setting {len(PARAMS)} parameters...")
        set_mode(m, 'FBWA')
        for name, val in PARAMS.items():
            set_param(m, name, val)

        # ── Phase 1: Standby on rail ───────────────────────────────────────
        real_p1 = STANDBY_S / SPEEDUP
        print(f"\n── Phase 1: Rail standby ({STANDBY_S}s sim / {real_p1:.1f}s real) ──")
        print(f"   AHRS_TRIM_Y={AHRS_TRIM_Y_RAIL:.4f} rad → FC sees +{CATAPULT_PITCH_DEG}° pitch")
        print(f"   Throttle idle (1300), NOT armed, neutral elevator")
        drain(m, real_p1, throttle_pwm=1300, elevator_pwm=1500)

        # ── Phase 2: Catapult launch ───────────────────────────────────────
        real_p2 = LAUNCH_S / SPEEDUP
        print(f"\n── Phase 2: Launch ({LAUNCH_S}s sim / {real_p2:.1f}s real) ──")
        print(f"   Resetting AHRS_TRIM_Y → 0.0 (clean flight physics)")
        set_param(m, 'AHRS_TRIM_Y', 0.0)
        print(f"   Arming + full throttle (2000 PWM)...")
        armed = arm_force(m)
        print(f"   Armed: {armed}")
        drain(m, real_p2, throttle_pwm=2000, elevator_pwm=1500)

        # ── Phase 3: Cruise ────────────────────────────────────────────────
        real_p3 = CRUISE_S / SPEEDUP
        print(f"\n── Phase 3: Cruise ({CRUISE_S}s sim / {real_p3:.1f}s real) ──")
        print(f"   Throttle cruise (1600 PWM), FBWA holds attitude")
        drain(m, real_p3, throttle_pwm=1600, elevator_pwm=1500)

        print(f"\n  Stopping SITL...")
    finally:
        stop_sitl(proc)

    # ── Find and parse log ─────────────────────────────────────────────────
    log_dir = os.path.join(WORK_DIR, 'logs')
    logs = sorted(
        [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith('.BIN')],
        key=os.path.getmtime)
    if not logs:
        print("ERROR: no BIN log found")
        sys.exit(1)
    log_path = logs[-1]
    print(f"\n  Parsing log: {log_path}")

    pidp, att, imu = read_log(log_path)
    if not pidp:
        print("ERROR: no PIDP data in log")
        sys.exit(1)
    if not att:
        print("ERROR: no ATT data in log")
        sys.exit(1)
    if not imu:
        print("ERROR: no IMU data in log")
        sys.exit(1)

    # Locate launch moment
    launch_us = find_launch_time_us(imu, threshold=TKOFF_MINACC)
    if launch_us is None:
        print("  WARNING: AccX never exceeded threshold — Fix #1 may not have fired")
        launch_us = int(STANDBY_S * 1e6)   # fall back to nominal time

    print(f"  Launch detected at T={launch_us/1e6:.2f}s")

    # Phase 1: I before launch
    i_p1 = [abs(I) for ts, I in pidp if ts < launch_us]
    p1_max_i = max(i_p1) if i_p1 else 0.0

    # Phase 2: AccX max, I within 1 sim-second after launch
    max_accx = max((ax for _, ax in imu), default=0.0)
    window_us = 1_000_000   # 1 simulated second in µs
    i_after = [abs(I) for ts, I in pidp
               if launch_us < ts <= launch_us + window_us]
    p2_i_max_after = max(i_after) if i_after else 0.0

    # Phase 3: pitch after the launch + climb window
    p3_start_us = launch_us + LAUNCH_S * 1_000_000
    pitch_p3 = [pitch for ts, pitch in att if ts >= p3_start_us]
    p3_min_pitch = min(pitch_p3) if pitch_p3 else 0.0
    p3_max_pitch = max(pitch_p3) if pitch_p3 else 0.0
    p3_avg_pitch = (sum(pitch_p3) / len(pitch_p3)) if pitch_p3 else 0.0

    # ── Verdicts ───────────────────────────────────────────────────────────
    p1_pass = p1_max_i < IMAX_DEG
    p2_pass = (max_accx > TKOFF_MINACC) and (p2_i_max_after < 1.0)
    p3_pass = p3_min_pitch > PITCH_MIN_DEG

    print(f"\n{'='*65}")
    print("RESULTS")
    print(f"{'='*65}")
    print(f"  Phase 1 — Integrator on rail:")
    print(f"    max |PIDP.I|  = {p1_max_i:.2f}°  (limit < {IMAX_DEG:.1f}°)")
    print(f"    → {'PASS ✓' if p1_pass else 'FAIL ✗  ← integrator saturated on rail'}")
    print()
    print(f"  Phase 2 — Launch detection (Fix #1):")
    print(f"    max AccX      = {max_accx:.2f} m/s²  (threshold {TKOFF_MINACC} m/s²)")
    print(f"    |I| 1s after  = {p2_i_max_after:.2f}°  (limit < 1.0°)")
    print(f"    → {'PASS ✓' if p2_pass else 'FAIL ✗  ← Fix #1 did not clear integrator'}")
    print()
    print(f"  Phase 3 — Cruise pitch:")
    print(f"    min pitch     = {p3_min_pitch:.2f}°  (limit > {PITCH_MIN_DEG}°)")
    print(f"    max pitch     = {p3_max_pitch:.2f}°")
    print(f"    avg pitch     = {p3_avg_pitch:.2f}°")
    print(f"    → {'PASS ✓' if p3_pass else 'FAIL ✗  ← nose-dive detected'}")
    print()
    overall = p1_pass and p2_pass and p3_pass
    if overall:
        print("  OVERALL: PASS — plane does not pitch down after catapult launch ✓")
    else:
        print("  OVERALL: FAIL")
        if not p3_pass:
            print(f"    Nose-dive: pitch reached {p3_min_pitch:.1f}° "
                  f"(>{abs(PITCH_MIN_DEG):.0f}° nose-down).")
            print(f"    This indicates integrator windup carried a nose-down kick into flight.")

    return overall, log_path


if __name__ == '__main__':
    ok, log_path = main()
    sys.exit(0 if ok else 1)
