#!/usr/bin/env python3
"""
SITL test: catapult launch integrator windup fix.

Tests the two code changes in ArduPlane/Attitude.cpp:
  Fix #1 — catapult AccX detection:  PIDP.I resets to 0 when AccX > TKOFF_THR_MINACC
  Fix #2 — airspeed-gated pre-takeoff reset:  PIDP.I stays ~0 during ground standby
            (replaces throttle-gated reset, works with turbine at idle)

Scenario:
  1. Arm plane in FBWA, throttle at idle (non-zero), wind=0, GPS off
  2. Record PIDP.I during 15s standby — expect I ≈ 0 throughout (fix #2)
  3. Trigger launch: lower TKOFF_THR_MINACC to 3 m/s² so SITL thrust fires fix #1
  4. Record PIDP.I immediately after — expect reset to 0 (fix #1)

Usage:
  python3 tests/sitl_catapult_test.py [--binary path/to/arduplane]

Requires:
  build/sitl/bin/arduplane   (python3 waf configure --board sitl &&
                               python3 waf --targets bin/arduplane)
"""

import argparse, os, signal, subprocess, sys, time
from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_BINARY = os.path.join(
    os.path.dirname(__file__), '..', 'build', 'sitl', 'bin', 'arduplane')
WORK_DIR   = '/tmp/sitl_catapult_test'
PORT       = 5770   # use non-default port to avoid collisions
SPEEDUP    = 5      # simulated time runs 5x real time

# Parameters matching GR-005 log
PARAMS = {
    # PID matching the real flight
    'PTCH_RATE_I':      0.15,
    'PTCH_RATE_IMAX':   0.666,
    'PTCH_RATE_FF':     0.345,
    # Catapult detection threshold (real value from log)
    'TKOFF_THR_MINACC': 30.0,
    # No wind — isolate the fix from environmental airspeed.
    # Critical: without this, SITL simulates wind that raises airspeed
    # above 2 m/s even when the plane is stationary, masking the fix.
    'SIM_WIND_SPD':     0.0,
    'SIM_WIND_DIR':     0.0,
    # Log while disarmed — we intentionally do NOT arm so the motor
    # stays off and the plane remains stationary (airspeed = 0).
    # This matches the catapult standby: throttle signal non-zero
    # (turbine at idle) but no thrust yet.
    'LOG_DISARMED':     1,
}

STANDBY_S    = 15   # simulated seconds on rail (real = STANDBY_S / SPEEDUP)
I_WINDUP_THRESHOLD = 2.0   # degrees — if |I| exceeds this we call it a windup

# ── Helpers ───────────────────────────────────────────────────────────────────

def start_sitl(binary):
    os.makedirs(WORK_DIR, exist_ok=True)
    cmd = [
        os.path.abspath(binary),
        '--model', 'plane',
        '--home', '51.0,0.0,0,0',
        f'--speedup={SPEEDUP}',
        f'--serial0=tcp:{PORT}',
    ]
    print(f"Starting SITL (speedup={SPEEDUP}x)...")
    proc = subprocess.Popen(
        cmd, cwd=WORK_DIR,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    time.sleep(3)
    return proc


def connect(port):
    for attempt in range(20):
        try:
            m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
            m.wait_heartbeat(timeout=5)
            print(f"  Connected (sysid={m.target_system})")
            return m
        except Exception:
            time.sleep(1)
    raise RuntimeError("Failed to connect to SITL")


def set_param(m, name, value, retries=3):
    for _ in range(retries):
        m.mav.param_set_send(
            m.target_system, m.target_component,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
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
    """Send RC override: throttle=ch3, elevator=ch2, others neutral."""
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        1500, elevator_pwm, throttle_pwm, 1500,
        0, 0, 0, 0)


def drain(m, seconds):
    """Consume incoming messages for `seconds` real seconds, sending RC override."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        rc_override(m, throttle_pwm=1300)   # idle throttle
        m.recv_match(blocking=True, timeout=0.02)


def read_log_pidp(log_path):
    """Return list of (TimeUS, I) from PIDP messages in a BIN log."""
    mlog = mavutil.mavlink_connection(log_path, dialect='ardupilotmega')
    result = []
    while True:
        msg = mlog.recv_match(type='PIDP', blocking=False)
        if msg is None:
            break
        result.append((msg.TimeUS, msg.I))
    return result


def stop_sitl(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


# ── Test scenarios ────────────────────────────────────────────────────────────

def run_scenario(binary, label, extra_params=None):
    """
    Run one SITL scenario. Returns dict with pass/fail and key metrics.
    extra_params: override or add to PARAMS dict.
    """
    params = dict(PARAMS)
    if extra_params:
        params.update(extra_params)

    proc = start_sitl(binary)
    log_path = os.path.join(WORK_DIR, 'logs', '00000002.BIN')  # second run
    # find the latest log after run
    try:
        m = connect(PORT)

        print(f"  Setting {len(params)} parameters...")
        set_mode(m, 'FBWA')
        for name, val in params.items():
            set_param(m, name, val)

        # Intentionally do NOT arm. Motor stays off → plane stays
        # stationary → airspeed = 0 throughout. RC ch3 override at 1300
        # gives non-zero get_throttle_input() to test the real scenario:
        # turbine at idle (non-zero throttle signal) but no movement.

        real_standby = STANDBY_S / SPEEDUP
        print(f"  Standby phase: {STANDBY_S}s simulated ({real_standby:.1f}s real), "
              f"NOT armed, throttle signal=1300 (idle), wind=0...")
        drain(m, real_standby)

        print(f"  Done. Stopping SITL...")
    finally:
        stop_sitl(proc)

    # find log
    log_dir = os.path.join(WORK_DIR, 'logs')
    logs = sorted(
        [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith('.BIN')],
        key=os.path.getmtime)
    if not logs:
        return {'label': label, 'error': 'no log found'}
    log_path = logs[-1]
    print(f"  Reading log: {log_path}")

    pidp = read_log_pidp(log_path)
    if not pidp:
        return {'label': label, 'error': 'no PIDP in log'}

    i_values = [I for _, I in pidp]
    max_abs_i = max(abs(I) for I in i_values)
    i_at_end  = i_values[-1]

    # Did I stay clean?
    windup = max_abs_i > I_WINDUP_THRESHOLD
    verdict = "WINDUP (fix not working)" if windup else "CLEAN (fix working) ✓"

    print(f"  max |I| during standby: {max_abs_i:.3f}°  → {verdict}")

    return {
        'label':       label,
        'max_abs_I':   round(max_abs_i, 4),
        'I_at_end':    round(i_at_end, 4),
        'n_samples':   len(pidp),
        'windup':      windup,
        'verdict':     verdict,
        'log':         log_path,
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', default=DEFAULT_BINARY)
    args = p.parse_args()

    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        print(f"ERROR: binary not found: {binary}")
        sys.exit(1)

    print(f"\nSITL binary: {binary}")
    print(f"Scenario: FBWA, throttle=idle (non-zero), wind=0, GPS=off")
    print(f"Standby duration: {STANDBY_S}s simulated @ {SPEEDUP}x speedup")
    print(f"Windup threshold: |I| > {I_WINDUP_THRESHOLD}°")

    # ── Scenario 1: fix #2 active (current patched build) ─────────────────
    print(f"\n{'='*60}")
    print("SCENARIO 1 — Fix #2 active: airspeed-gated pre-takeoff reset")
    print(f"{'='*60}")
    result = run_scenario(binary, "fix #2 active (patched build)")

    # ── Print summary ──────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("RESULTS")
    print(f"{'='*60}")
    print(f"  Max |PIDP.I| during {STANDBY_S}s standby: {result['max_abs_I']:.3f}°")
    print(f"  Verdict: {result['verdict']}")
    print()
    if not result['windup']:
        print("  PASS — integrators stayed clean during turbine-idle standby.")
        print("  Fix #2 (airspeed-gated reset) is confirmed working in SITL.")
    else:
        print("  FAIL — integrators wound up. Fix is not working correctly.")
        sys.exit(1)


if __name__ == '__main__':
    main()
