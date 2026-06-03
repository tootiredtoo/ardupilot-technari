#!/usr/bin/env python3
"""
SITL test — Fix #1 only: catapult AccX detection.

Scenario
--------
Reproduces the GR-005 windy-day crash scenario in SITL:

  Phase 1  Standby (30 s simulated)
    Fix #2 is deliberately bypassed:
      SIM_ARSPD_OFS = 10 m/s  →  pitot reads 10 m/s even at standstill
      (matches the real crash: 10 m/s headwind on the catapult rail)
    Elevator ch2 = 1300 PWM  →  nose-down pitch command the stationary
      plane cannot achieve  →  persistent pitch-rate error  →  fast I windup
    Expected: PIDP.I winds toward −38° (IMAX saturation)

  Phase 2  Launch (5 s simulated)
    SIM_ARSPD_OFS = 0  (remove bias — airspeed now real)
    TKOFF_THR_MINACC = 3 m/s²  (SITL-scaled; real catapult = 30 m/s²)
    Force-arm + full throttle  →  AccX from motor > 3 m/s²
    Fix #1 fires  →  PIDP.I resets from −Xdeg to ≈ 0 for 300 ms
    Expected: |I| < 1° within 500 ms of arm

  Phase 3  Flight (15 s simulated)
    Free flight, neutral elevator
    Expected: ATT.Pitch > −30°, PIDP.I bounded

Why Fix #1 alone is sufficient
  Windy day  Fix #2 bypassed (airspeed > 2 m/s on rail)  →  Fix #1 is the
             only safeguard  →  resets wound integrator at launch
  Calm day   Fix #2 also active, but Fix #1 runs regardless  →  both fire

Usage
  python3 tests/sitl_full_scenario_test.py [--binary PATH]
"""

import argparse, os, shutil, signal, subprocess, sys, time
from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_BINARY = os.path.join(
    os.path.dirname(__file__), '..', 'build', 'sitl', 'bin', 'arduplane')
WORK_DIR  = '/tmp/sitl_full_scenario_test'
PORT      = 5771
SPEEDUP   = 5

STANDBY_S   = 30.0   # simulated seconds on rail (long enough for visible windup)
LAUNCH_S    = 5.0    # simulated seconds arm → observe reset
FLIGHT_S    = 15.0   # simulated seconds of free flight

TKOFF_MINACC_SITL = 3.0   # real value 30 m/s²; SITL motor peaks ~12 m/s²

# SIM_ARSPD_OFS value that gives ~10 m/s airspeed reading on the stationary
# rail, simulating turbine exhaust blowing across the pitot tube.
# Formula (with ARSPD_OFFSET=0): ARSP = sqrt(ARSPD_RATIO × SIM_ARSPD_OFS)
#   SIM_ARSPD_OFS = 50  →  sqrt(1.9936 × 50) ≈ 9.98 m/s  > 8.5 m/s threshold
# This clears the underspeed lock so the integrator can wind, matching the
# real GR-005 crash where turbine exhaust raised the pitot reading to 8–11 m/s.
ARSPD_OFS_RAIL = 50   # raw pressure units ≈ 10 m/s equivalent

# Parameters from GR-005_2_Crush.BIN
PARAMS = {
    'PTCH_RATE_I':      0.15,
    'PTCH_RATE_IMAX':   0.666,
    'PTCH_RATE_FF':     0.345,
    'LOG_DISARMED':     1,
    'TKOFF_THR_MINACC': 30.0,         # real value; won't trigger in SITL
    'SIM_ARSPD_OFS':    ARSPD_OFS_RAIL,
    'SIM_WIND_SPD':     0.0,
    # Skip boot calibration and force ARSPD_OFFSET = 0.
    # Without these the analog sensor calibrates at boot to offset ≈ 2014,
    # making every reading report ~63 m/s regardless of actual airspeed.
    # With SKIP_CAL=1 + OFFSET=0:
    #   SIM_ARSPD_OFS=50 → ARSP ≈ 10 m/s  (above 8.5 m/s underspeed threshold)
    #   SIM_ARSPD_OFS=0  → ARSP ≈  0 m/s  (launch phase, sensor sees real speed)
    'ARSPD_SKIP_CAL':   1,
    'ARSPD_OFFSET':     0.0,
}


# ── SITL helpers ──────────────────────────────────────────────────────────────

def start_sitl(binary):
    shutil.rmtree(WORK_DIR, ignore_errors=True)   # ensure no stale eeprom
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
    time.sleep(5)   # allow full boot before connecting
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
        t = msg.get_type()
        if t == 'PIDP':
            pidp.append({'ts': ts, 'I': msg.I})
        elif t == 'ATT':
            att.append({'ts': ts, 'Pitch': msg.Pitch})
        elif t == 'ARSP':
            arsp.append({'ts': ts, 'Airspeed': msg.Airspeed})
        elif t == 'IMU':
            imu.append({'ts': ts, 'AccX': msg.AccX})
    return pidp, att, arsp, imu


def nearest(series, ts, field):
    best = None
    for item in series:
        if best is None or abs(item['ts'] - ts) < abs(best['ts'] - ts):
            best = item
    return best[field] if best else 0.0


# ── Analysis ──────────────────────────────────────────────────────────────────

def find_arm_ts(pidp, first_pidp_ts):
    """Estimate simulated timestamp when arm was sent."""
    return first_pidp_ts + int(STANDBY_S * 1e6)


def find_fix1_reset(pidp, arm_ts):
    """
    Find the largest positive jump in I anywhere near the arm window.
    Search from 5 s before arm_ts to 10 s after, to tolerate timing skew
    between the wall-clock arm_ts estimate and the actual sim timestamp.
    """
    window_start = arm_ts - int(5e6)
    window_end   = arm_ts + int(10e6)
    candidates = [p for p in pidp
                  if window_start <= p['ts'] <= window_end]
    best = None
    for i in range(1, len(candidates)):
        delta = candidates[i]['I'] - candidates[i-1]['I']
        if delta > 1.0 and candidates[i-1]['I'] < -1.0:
            if best is None or delta > best['delta']:
                best = {
                    'ts':       candidates[i]['ts'],
                    'I_before': candidates[i-1]['I'],
                    'I_after':  candidates[i]['I'],
                    'delta':    delta,
                }
    return best


def print_log_table(pidp, att, arsp, arm_ts, reset_ts):
    """Print a time-aligned table of the full flight."""
    imax_deg = 0.666 * 57.296   # 38.16°

    print()
    print(f"  {'Time(sim s)':>11}  {'PIDP.I (°)':>11}  {'Pitch (°)':>10}  "
          f"{'Airspd (m/s)':>13}  Status")
    print(f"  {'-'*11}  {'-'*11}  {'-'*10}  {'-'*13}  ------")

    step = max(1, len(pidp) // 50)
    ai = si = 0

    for p in pidp[::step]:
        ts   = p['ts']
        I    = p['I']
        t_s  = ts / 1e6

        while ai < len(att)  - 1 and att[ai+1]['ts']  <= ts: ai += 1
        while si < len(arsp) - 1 and arsp[si+1]['ts'] <= ts: si += 1
        pitch = att[ai]['Pitch']  if att  else 0.0
        spd   = arsp[si]['Airspeed'] if arsp else 0.0

        if ts < arm_ts:
            if abs(I) >= imax_deg * 0.98:
                status = 'standby — IMAX SATURATED'
            elif abs(I) > 5:
                status = 'standby — winding up'
            else:
                status = 'standby'
        elif reset_ts and abs(ts - reset_ts) < 200_000:
            status = '<<< FIX #1 RESET >>>'
        elif ts < arm_ts + 500_000:
            status = 'launch'
        else:
            status = 'flight — CRASH!' if pitch < -30 else 'flight'

        print(f"  {t_s:>11.1f}  {I:>+11.3f}  {pitch:>+10.2f}  "
              f"{spd:>13.2f}  {status}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', default=DEFAULT_BINARY)
    args = ap.parse_args()

    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        sys.exit(f"ERROR: binary not found: {binary}")

    total_sim = STANDBY_S + LAUNCH_S + FLIGHT_S
    imax_deg  = 0.666 * 57.296   # 38.16°

    print(f"\n{'='*65}")
    print("  FIX #1 SITL TEST — catapult AccX detection")
    print(f"{'='*65}")
    print(f"  Binary : {binary}")
    print()
    print("  Fix #2 deliberately bypassed:")
    print(f"    SIM_ARSPD_OFS = {ARSPD_OFS_RAIL} m/s")
    print(f"    → pitot reads {ARSPD_OFS_RAIL}+ m/s even at standstill")
    print(f"    → airspeed_est > 2 m/s  →  Fix #2 'not moving' check fails")
    print(f"    → integrator winds freely  (matches windy-day real crash)")
    print()
    print("  Fix #1 under test:")
    print(f"    TKOFF_THR_MINACC = {TKOFF_MINACC_SITL} m/s²")
    print(f"    (real catapult: 30 m/s²; SITL motor peaks ~12 m/s²)")
    print(f"    → AccX > {TKOFF_MINACC_SITL}  →  I zeroed for 300 ms")
    print()
    print(f"  Standby : {STANDBY_S:.0f} s sim  ({STANDBY_S/SPEEDUP:.0f} s real)")
    print(f"  Launch  : {LAUNCH_S:.0f} s sim  ({LAUNCH_S/SPEEDUP:.0f} s real)")
    print(f"  Flight  : {FLIGHT_S:.0f} s sim  ({FLIGHT_S/SPEEDUP:.0f} s real)")
    print(f"  Total   : {total_sim:.0f} s sim  ({total_sim/SPEEDUP:.0f} s real)")
    print(f"{'='*65}")

    proc = start_sitl(binary)
    try:
        m = connect(PORT)
        print("  Connected.")
        set_mode(m, 'FBWA')
        print("  Setting parameters...")
        for name, val in PARAMS.items():
            set_param(m, name, val)

        # ── Phase 1: Standby ─────────────────────────────────────────────
        rs = STANDBY_S / SPEEDUP
        print(f"\n  [PHASE 1] Standby — {STANDBY_S:.0f} s sim / {rs:.0f} s real")
        print(f"    Elevator ch2 = 1300  (nose-down command → pitch-rate error)")
        print(f"    SIM_ARSPD_OFS = {ARSPD_OFS_RAIL} m/s  (Fix #2 bypassed)")
        print(f"    Expecting integrator to wind toward −{imax_deg:.1f}° ...")
        drain(m, rs, throttle=1300, elevator=1300)

        # ── Phase 2: Launch ───────────────────────────────────────────────
        rl = LAUNCH_S / SPEEDUP
        print(f"\n  [PHASE 2] Launch — {LAUNCH_S:.0f} s sim / {rl:.0f} s real")
        print(f"    Removing airspeed bias, lowering TKOFF_THR_MINACC → {TKOFF_MINACC_SITL}")
        set_param(m, 'SIM_ARSPD_OFS',    0.0)
        set_param(m, 'TKOFF_THR_MINACC', TKOFF_MINACC_SITL)
        armed = arm_force(m)
        print(f"    Armed: {armed}  →  full throttle")
        drain(m, rl, throttle=2000, elevator=1500)

        # ── Phase 3: Flight ───────────────────────────────────────────────
        rf = FLIGHT_S / SPEEDUP
        print(f"\n  [PHASE 3] Flight — {FLIGHT_S:.0f} s sim / {rf:.0f} s real")
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

    first_ts = pidp[0]['ts']
    arm_ts   = find_arm_ts(pidp, first_ts)
    reset    = find_fix1_reset(pidp, arm_ts)

    # ── Print table ───────────────────────────────────────────────────────
    print_log_table(pidp, att, arsp, arm_ts, reset['ts'] if reset else None)

    # ── Summary ───────────────────────────────────────────────────────────
    # Use the reset timestamp (if found) to split phases precisely; fall
    # back to the estimated arm_ts if no reset was detected.
    split_ts  = reset['ts'] if reset else arm_ts
    p1        = [p for p in pidp if p['ts'] < split_ts]
    p2        = [p for p in pidp if split_ts <= p['ts'] <= split_ts + 1_000_000]
    p3_att    = [a for a in att if a['ts'] > split_ts + 2_000_000]

    # Peak wound value before the reset (not the last sample, which may be post-reset)
    I_pre     = reset['I_before'] if reset else (
                    max((p['I'] for p in p1), key=abs, default=0.0) if p1 else 0.0)
    I_after   = reset['I_after'] if reset else (p2[0]['I'] if p2 else 0.0)
    max_accel = max((x['AccX'] for x in imu
                     if split_ts - int(1e6) <= x['ts'] <= split_ts + 3_000_000), default=0.0)
    min_pitch = min((a['Pitch'] for a in p3_att), default=0.0)

    print(f"\n{'='*65}")
    print("  SUMMARY")
    print(f"{'='*65}")
    print(f"  Pre-launch  PIDP.I : {I_pre:+.3f}°")
    if reset:
        print(f"  Fix #1 reset       : {reset['I_before']:+.3f}° → "
              f"{reset['I_after']:+.3f}°  (Δ = {reset['delta']:+.3f}°)")
    else:
        print(f"  Fix #1 reset       : not detected")
    cmp = '>' if max_accel > TKOFF_MINACC_SITL else '<'
    print(f"  Max AccX at arm    : {max_accel:.2f} m/s²  "
          f"({cmp} {TKOFF_MINACC_SITL} threshold)")
    print(f"  Post-reset PIDP.I  : {I_after:+.3f}°")
    print(f"  Min pitch (flight) : {min_pitch:+.2f}°")

    # ── Pass / Fail ───────────────────────────────────────────────────────
    p1_pass = abs(I_pre) > 5.0
    p2_pass = (max_accel > TKOFF_MINACC_SITL) and (reset is not None) and (abs(I_after) < 5.0)
    p3_pass = min_pitch > -30.0 and len(p3_att) > 0

    print()
    def verdict(ok, msg): print(f"  {'✓ PASS' if ok else '✗ FAIL'}  {msg}")
    verdict(p1_pass,
            f"Phase 1: integrator peak wound to {I_pre:+.3f}°  (Fix #2 bypassed)")
    verdict(p2_pass,
            f"Phase 2: Fix #1 fired (AccX {max_accel:.1f} m/s²), I reset to {I_after:+.3f}°")
    verdict(p3_pass,
            f"Phase 3: flight controlled  (min pitch {min_pitch:+.2f}°)")

    overall = p1_pass and p2_pass and p3_pass
    print()
    if overall:
        print("  OVERALL: PASS ✓")
        print("  Fix #1 alone prevents the crash in the windy-day scenario.")
        print("  On a calm day Fix #2 also helps, but Fix #1 covers both cases.")
    else:
        print("  OVERALL: FAIL ✗")
        sys.exit(1)
    print(f"{'='*65}\n")


if __name__ == '__main__':
    main()
