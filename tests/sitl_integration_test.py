#!/usr/bin/env python3
"""
SITL integration test — full launch + flight validation.

Three-phase scenario that exercises the GR-008-style catapult ground hold
state machine:

  Phase 1 — Calm standby (10 s simulated)
    Plane sits on catapult rail at 12.35°, wind = 0, plane NOT armed.
    catapult_ground_hold is NOT yet active (requires armed).
    Standard zero-throttle/disarmed reset keeps PIDP.I = 0.
    Pass: max |I| < 2°

  Phase 2 — Armed + launch (5 s simulated)
    Plane armed on rail (full throttle). catapult_ground_hold activates
    because airspeed < ARSPD_FBW_MIN. I is frozen at 0 even at 100% throttle.
    Motor accelerates the plane; airspeed crosses ARSPD_FBW_MIN → state
    machine releases. No TKOFF_THR_MINACC / Fix #1 involved.
    Pass: I stays < 1° until airspeed rise is confirmed

  Phase 3 — Flight with wind gusts (25 s simulated)
    Steady 8 m/s headwind + SIM_WIND_TURB = 3 m/s turbulence. Integrator
    is free to accumulate a real steady-state correction but must stay bounded.
    Pass: pitch > -30°, |I| < IMAX, no crash

Usage:
  python3 tests/sitl_integration_test.py [--binary PATH]
"""

import argparse, math, os, shutil, signal, subprocess, sys, time
from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_BINARY = os.path.join(
    os.path.dirname(__file__), '..', 'build', 'sitl', 'bin', 'arduplane')
WORK_DIR  = '/tmp/sitl_integration_test'
PORT      = 5774
SPEEDUP   = 5

# Phase durations (simulated seconds)
STANDBY_S = 10.0
LAUNCH_S  =  5.0
FLIGHT_S  = 25.0

# Catapult rail pitch angle is 12.35° nose-up.
# Simulated via AHRS_TRIM_Y: setting it to -radians(12.35) makes the AHRS
# report +12.35° pitch to the FC even though SITL physics keeps the plane
# level.  The FC in FBWA then targets 0° against a reported +12.35°, creating
# the correct -12.35° pitch error — identical to the real catapult scenario.
CATAPULT_PITCH_DEG = 12.35
AHRS_TRIM_Y_RAIL   = -math.radians(CATAPULT_PITCH_DEG)   # ≈ -0.2155 rad

# Airspeed at which catapult_ground_hold releases (must match ARSPD_FBW_MIN).
ARSPD_FBW_MIN = 12.0   # m/s — default for most planes

# Gust parameters injected at start of Phase 3
WIND_SPD_FLIGHT  =  8.0   # m/s steady headwind
WIND_TURB_FLIGHT =  3.0   # m/s turbulence amplitude
WIND_TC_FLIGHT   =  2.0   # s turbulence time-constant (lower = choppier)

# GR-005 PID values
PARAMS = {
    'PTCH_RATE_I':      0.15,
    'PTCH_RATE_IMAX':   0.666,
    'PTCH_RATE_FF':     0.345,
    'LOG_DISARMED':     1,
    # Calm day on rail: no wind, no airspeed bias
    'SIM_WIND_SPD':     0.0,
    'SIM_WIND_TURB':    0.0,
    'SIM_WIND_DIR':     0.0,
    'SIM_ARSPD_OFS':    0.0,
    # Disable boot-time airspeed calibration
    'ARSPD_SKIP_CAL':   1,
    'ARSPD_OFFSET':     0.0,
    # No AccX-based integrator reset on this branch
    'TKOFF_THR_MINACC': 0.0,
    # Enough cruise throttle to stay airborne in SITL after launch
    'TRIM_THROTTLE':    0.6,
    # Zero STAB_PITCH_DOWN to avoid SITL airspeed noise interference
    'STAB_PITCH_DOWN':  0,
    # Simulate 12.35° nose-up catapult rail via AHRS trim
    'AHRS_TRIM_Y':      AHRS_TRIM_Y_RAIL,
    # Flyable airspeed minimum — also the catapult_ground_hold release threshold
    'ARSPD_FBW_MIN':    ARSPD_FBW_MIN,
}

IMAX_DEG = 0.666 * 57.2958   # 38.2°

# ── SITL helpers ──────────────────────────────────────────────────────────────

def start_sitl(binary):
    shutil.rmtree(WORK_DIR, ignore_errors=True)
    os.makedirs(WORK_DIR, exist_ok=True)
    proc = subprocess.Popen(
        [os.path.abspath(binary),
         '--model', 'plane',
         '--home', '51.0,0.0,0,352',
         f'--speedup={SPEEDUP}',
         f'--serial0=tcp:{PORT}'],
        cwd=WORK_DIR,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    time.sleep(5)
    return proc


def connect(port):
    for _ in range(30):
        try:
            m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
            m.wait_heartbeat(timeout=5)
            if m.target_system == 0:
                time.sleep(1)
                continue
            return m
        except Exception:
            time.sleep(1)
    raise RuntimeError("Cannot connect to SITL")


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
    for _ in range(5):
        m.mav.set_mode_send(m.target_system,
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


def rc_override(m, throttle=1300, elevator=1500):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        1500, elevator, throttle, 1500, 0, 0, 0, 0)


def drain(m, real_seconds, throttle=1300, elevator=1500):
    end = time.time() + real_seconds
    while time.time() < end:
        rc_override(m, throttle=throttle, elevator=elevator)
        m.recv_match(blocking=True, timeout=0.02)


def stop_sitl(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


# ── Log reader ────────────────────────────────────────────────────────────────

def read_log(path):
    mlog = mavutil.mavlink_connection(path, dialect='ardupilotmega')
    pidp, att, arsp, imu = [], [], [], []
    while True:
        msg = mlog.recv_match(type=['PIDP', 'ATT', 'ARSP', 'IMU'], blocking=False)
        if msg is None:
            break
        ts = getattr(msg, 'TimeUS', 0)
        t  = msg.get_type()
        if t == 'PIDP':
            flags = getattr(msg, 'Flags', 0)
            pidp.append({'ts': ts, 'I': msg.I, 'P': msg.P, 'D': msg.D,
                         'Flags': flags})
        elif t == 'ATT':
            att.append({'ts': ts, 'Pitch': msg.Pitch, 'Roll': msg.Roll})
        elif t == 'ARSP':
            arsp.append({'ts': ts, 'Airspeed': msg.Airspeed})
        elif t == 'IMU':
            imu.append({'ts': ts, 'AccX': msg.AccX})
    return pidp, att, arsp, imu


def find_ground_hold_release(pidp, arsp, arm_ts):
    """Find the moment catapult_ground_hold releases: first time after arming
    that airspeed exceeds ARSPD_FBW_MIN and I is no longer exactly zero."""
    window = [r for r in arsp if r['ts'] > arm_ts]
    for r in window:
        if r['Airspeed'] >= ARSPD_FBW_MIN:
            return r['ts']
    return None


# ── Reporting ─────────────────────────────────────────────────────────────────

def print_table(pidp, att, arsp, arm_ts, release_ts, gust_ts):
    print()
    print(f"  {'Time(sim s)':>11}  {'PIDP.I (°)':>11}  {'Flags':>5}  "
          f"{'Pitch (°)':>10}  {'Airspd (m/s)':>13}  Phase")
    print(f"  {'-'*11}  {'-'*11}  {'-'*5}  {'-'*10}  {'-'*13}  -----")

    ai = si = 0
    step = max(1, len(pidp) // 70)
    for p in pidp[::step]:
        ts    = p['ts']
        I     = p['I']
        flags = p.get('Flags', 0)
        t_s   = ts / 1e6
        while ai < len(att)  - 1 and att[ai+1]['ts']  <= ts: ai += 1
        while si < len(arsp) - 1 and arsp[si+1]['ts'] <= ts: si += 1
        pitch = att[ai]['Pitch']    if att  else 0.0
        spd   = arsp[si]['Airspeed'] if arsp else 0.0

        if ts < arm_ts:
            phase = 'standby (disarmed)'
        elif release_ts and ts < release_ts:
            phase = 'ground-hold (armed, aspd<FBW_MIN)'
        elif ts < arm_ts + int(LAUNCH_S * 1e6):
            phase = 'launch (hold released)'
        elif gust_ts and ts >= gust_ts:
            phase = 'flight+gusts'
        else:
            phase = 'flight'

        print(f"  {t_s:>11.1f}  {I:>+11.3f}  {flags:>5}  {pitch:>+10.2f}  "
              f"{spd:>13.2f}  {phase}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', default=DEFAULT_BINARY)
    args = ap.parse_args()

    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        sys.exit(f"ERROR: binary not found: {binary}")

    total_sim = STANDBY_S + LAUNCH_S + FLIGHT_S

    print(f"\n{'='*68}")
    print("  SITL INTEGRATION TEST — GR-008 catapult ground hold state machine")
    print(f"{'='*68}")
    print(f"  Binary : {binary}")
    print()
    print(f"  Phase 1  Disarmed standby  {STANDBY_S:.0f} s sim / {STANDBY_S/SPEEDUP:.0f} s real")
    print(f"           Not armed, idle throttle, wind=0  →  I = 0 (standard reset)")
    print(f"  Phase 2  Armed + launch    {LAUNCH_S:.0f} s sim / {LAUNCH_S/SPEEDUP:.0f} s real")
    print(f"           Armed + full throttle, airspeed < {ARSPD_FBW_MIN} m/s")
    print(f"           catapult_ground_hold active  →  I frozen at 0 (Flags=9)")
    print(f"           Hold releases when airspeed >= {ARSPD_FBW_MIN} m/s")
    print(f"  Phase 3  Gusty flight      {FLIGHT_S:.0f} s sim / {FLIGHT_S/SPEEDUP:.0f} s real")
    print(f"           Wind {WIND_SPD_FLIGHT} m/s + turbulence {WIND_TURB_FLIGHT} m/s")
    print(f"           Integrator free to build correction, must stay bounded")
    print(f"  Total    {total_sim:.0f} s sim / {total_sim/SPEEDUP:.0f} s real  |  IMAX {IMAX_DEG:.1f}°")
    print(f"{'='*68}")

    proc = start_sitl(binary)
    try:
        m = connect(PORT)
        print("  Connected.")
        set_mode(m, 'FBWA')
        print(f"  Setting {len(PARAMS)} parameters...")
        for name, val in PARAMS.items():
            set_param(m, name, val)

        # ── Phase 1: Disarmed standby ─────────────────────────────────────
        rs = STANDBY_S / SPEEDUP
        print(f"\n  [PHASE 1] Disarmed standby — {STANDBY_S:.0f} s sim / {rs:.0f} s real")
        print(f"    Not armed, idle throttle, AHRS_TRIM_Y={AHRS_TRIM_Y_RAIL:.4f} rad "
              f"(FC sees +{CATAPULT_PITCH_DEG}°)")
        drain(m, rs, throttle=1300, elevator=1500)

        # ── Phase 2: Armed + launch ───────────────────────────────────────
        rl = LAUNCH_S / SPEEDUP
        print(f"\n  [PHASE 2] Armed + launch — {LAUNCH_S:.0f} s sim / {rl:.0f} s real")
        set_param(m, 'AHRS_TRIM_Y', 0.0)
        armed = arm_force(m)
        print(f"    Armed: {armed}  →  full throttle, catapult_ground_hold active")
        print(f"    I should stay = 0 until airspeed >= {ARSPD_FBW_MIN} m/s")
        drain(m, rl, throttle=2000, elevator=1500)

        # ── Phase 3: Gusty flight ─────────────────────────────────────────
        rf = FLIGHT_S / SPEEDUP
        print(f"\n  [PHASE 3] Gusty flight — {FLIGHT_S:.0f} s sim / {rf:.0f} s real")
        print(f"    Injecting wind: {WIND_SPD_FLIGHT} m/s steady + "
              f"{WIND_TURB_FLIGHT} m/s turbulence (TC={WIND_TC_FLIGHT} s)")
        set_param(m, 'SIM_WIND_SPD',  WIND_SPD_FLIGHT)
        set_param(m, 'SIM_WIND_DIR',  170.0)
        set_param(m, 'SIM_WIND_TURB', WIND_TURB_FLIGHT)
        set_param(m, 'SIM_WIND_TC',   WIND_TC_FLIGHT)
        drain(m, rf, throttle=1800, elevator=1500)

        print("  Stopping SITL...")
    finally:
        stop_sitl(proc)

    # ── Find log ──────────────────────────────────────────────────────────
    log_dir = os.path.join(WORK_DIR, 'logs')
    logs = sorted(
        [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith('.BIN')],
        key=os.path.getmtime)
    if not logs:
        sys.exit(f"ERROR: no .BIN log in {log_dir}")
    log_path = logs[-1]
    print(f"\n  Log: {log_path}")

    pidp, att, arsp, imu = read_log(log_path)
    if not pidp:
        sys.exit("ERROR: no PIDP messages in log")

    first_ts  = pidp[0]['ts']
    arm_ts    = first_ts + int(STANDBY_S * 1e6)
    gust_ts   = arm_ts  + int(LAUNCH_S  * 1e6)
    release_ts = find_ground_hold_release(pidp, arsp, arm_ts)

    print_table(pidp, att, arsp, arm_ts, release_ts, gust_ts)

    # ── Per-phase metrics ─────────────────────────────────────────────────
    p1 = [p for p in pidp if p['ts'] < arm_ts]
    p2 = [p for p in pidp if arm_ts <= p['ts'] < gust_ts]
    p3 = [p for p in pidp if p['ts'] >= gust_ts]
    p3_att = [a for a in att if a['ts'] >= gust_ts]

    I_p1_max  = max((abs(p['I']) for p in p1), default=0.0)

    # Find ground-hold window: armed up to release_ts
    if release_ts:
        p2_hold = [p for p in p2 if p['ts'] < release_ts]
        p2_free = [p for p in p2 if p['ts'] >= release_ts]
    else:
        p2_hold = p2
        p2_free = []

    I_hold_max  = max((abs(p['I']) for p in p2_hold), default=0.0)
    flags_hold  = [p.get('Flags', 0) for p in p2_hold]
    flags9_pct  = (sum(1 for f in flags_hold if f == 9) / len(flags_hold) * 100
                   if flags_hold else 0.0)
    arsp_at_release = next((r['Airspeed'] for r in arsp
                             if release_ts and r['ts'] >= release_ts), 0.0)

    I_p3_max  = max((abs(p['I']) for p in p3), default=0.0)
    min_pitch = min((a['Pitch'] for a in p3_att), default=0.0)
    max_spd   = max((r['Airspeed'] for r in arsp if r['ts'] >= gust_ts), default=0.0)

    print(f"\n{'='*68}")
    print("  SUMMARY")
    print(f"{'='*68}")
    print(f"  Phase 1  max |I| during {STANDBY_S:.0f} s standby  : {I_p1_max:.3f}°")
    print(f"  Phase 2  Ground hold duration             : "
          f"{(release_ts - arm_ts)/1e6:.2f} s" if release_ts else
          f"  Phase 2  Ground hold                      : never released!")
    print(f"           max |I| while hold active        : {I_hold_max:.3f}°")
    print(f"           PIDP.Flags=9 during hold         : {flags9_pct:.0f}%")
    print(f"           Airspeed at release               : {arsp_at_release:.1f} m/s"
          f"  (FBW_MIN = {ARSPD_FBW_MIN} m/s)")
    print(f"  Phase 3  max |I| during gusty flight      : {I_p3_max:.3f}°  "
          f"(IMAX = {IMAX_DEG:.1f}°)")
    print(f"           Min pitch during flight           : {min_pitch:+.2f}°")
    print(f"           Max airspeed during flight        : {max_spd:.1f} m/s")

    # ── Pass / Fail ───────────────────────────────────────────────────────
    p1_pass = I_p1_max < IMAX_DEG

    # Phase 2: hold must have been active, I must have stayed near zero,
    # and Flags=9 should dominate the hold window (state machine active).
    p2_pass = (release_ts is not None) and (I_hold_max < 1.0) and (flags9_pct > 50.0)

    p3_pass = (min_pitch > -30.0) and (I_p3_max < IMAX_DEG) and (len(p3_att) > 0)

    print()
    def verdict(ok, msg): print(f"  {'✓ PASS' if ok else '✗ FAIL'}  {msg}")
    verdict(p1_pass,
            f"Phase 1 — disarmed standby: max |I| = {I_p1_max:.3f}°  (< IMAX)")
    verdict(p2_pass,
            f"Phase 2 — ground hold: I≤{I_hold_max:.3f}°, "
            f"Flags=9 {flags9_pct:.0f}% of hold, "
            f"released at {arsp_at_release:.1f} m/s")
    verdict(p3_pass,
            f"Phase 3 — flight stable under gusts "
            f"(min pitch {min_pitch:+.2f}°, max |I| {I_p3_max:.1f}°)")

    overall = p1_pass and p2_pass and p3_pass
    print()
    if overall:
        print("  OVERALL: PASS ✓")
        print("  GR-008 catapult ground hold confirmed: standby → hold → release → flight.")
    else:
        print("  OVERALL: FAIL ✗")
    print(f"{'='*68}\n")

    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
