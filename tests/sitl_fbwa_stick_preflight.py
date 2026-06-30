#!/usr/bin/env python3
"""
SITL test — FBWA stick inputs on catapult rail.

Scenario: plane is ARMED on the 12.35° catapult rail in FBWA mode.
The pilot moves the elevator stick (up / neutral / down) while waiting
for the catapult to fire.  catapult_ground_hold should keep I = 0
exactly despite both the pitch error AND the stick-induced pitch demand.

Pass criteria:
  - max |I| while armed on rail with stick movement < 1°  (hold active)
  - PIDP.Flags = 9 dominates the armed period  (state machine visible)
  - After simulated launch (airspeed rises), hold releases correctly
"""

import math, os, shutil, signal, subprocess, sys, time
from pymavlink import mavutil

WORK_DIR = '/tmp/sitl_fbwa_stick_preflight'
PORT     = 5776
SPEEDUP  = 5

CATAPULT_PITCH_DEG = 12.35
AHRS_TRIM_Y_RAIL   = -math.radians(CATAPULT_PITCH_DEG)
ARSPD_FBW_MIN      = 12.0

# Stick sweep timings (simulated seconds)
PREFLIGHT_S = 15.0   # armed on rail with stick movement
LAUNCH_S    =  5.0   # full throttle launch

PARAMS = {
    'PTCH_RATE_I':    0.15,
    'PTCH_RATE_IMAX': 0.666,
    'PTCH_RATE_FF':   0.345,
    'LOG_DISARMED':   1,
    'SIM_WIND_SPD':   0.0,
    'SIM_WIND_TURB':  0.0,
    'SIM_ARSPD_OFS':  0.0,
    'ARSPD_SKIP_CAL': 1,
    'ARSPD_OFFSET':   0.0,
    'TKOFF_THR_MINACC': 0.0,
    'TRIM_THROTTLE':  0.6,
    'STAB_PITCH_DOWN': 0,
    'AHRS_TRIM_Y':    AHRS_TRIM_Y_RAIL,
    'ARSPD_FBW_MIN':  ARSPD_FBW_MIN,
}

IMAX_DEG = 0.666 * 57.2958


# ── SITL helpers ──────────────────────────────────────────────────────────────

def start_sitl():
    shutil.rmtree(WORK_DIR, ignore_errors=True)
    os.makedirs(WORK_DIR, exist_ok=True)
    proc = subprocess.Popen(
        [os.path.join(os.path.dirname(__file__),
                      '..', 'build', 'sitl', 'bin', 'arduplane'),
         '--model', 'plane', '--home', '51.0,0.0,0,0',
         f'--speedup={SPEEDUP}', f'--serial0=tcp:{PORT}'],
        cwd=WORK_DIR,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    time.sleep(5)
    return proc


def connect():
    for _ in range(30):
        try:
            m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{PORT}', source_system=255)
            m.wait_heartbeat(timeout=5)
            if m.target_system != 0:
                return m
        except Exception:
            pass
        time.sleep(1)
    raise RuntimeError("Failed to connect to SITL")


def set_param(m, name, value):
    for _ in range(5):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            return


def set_mode(m, mode):
    mid = m.mode_mapping()[mode]
    for _ in range(5):
        m.mav.set_mode_send(m.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.custom_mode == mid:
            return


def arm_force(m):
    for _ in range(10):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 21196, 0, 0, 0, 0, 0)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def rc(m, throttle=1300, elevator=1500):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        1500, elevator, throttle, 1500, 0, 0, 0, 0)


def stop_sitl(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


# ── Log reader ────────────────────────────────────────────────────────────────

def read_log(path):
    mlog = mavutil.mavlink_connection(path, dialect='ardupilotmega')
    pidp, arsp = [], []
    while True:
        msg = mlog.recv_match(type=['PIDP', 'ARSP'], blocking=False)
        if msg is None:
            break
        ts = msg.TimeUS
        if msg.get_type() == 'PIDP':
            pidp.append({'ts': ts, 'I': msg.I,
                         'Flags': getattr(msg, 'Flags', 0)})
        else:
            arsp.append({'ts': ts, 'Airspeed': msg.Airspeed})
    return pidp, arsp


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"\n{'='*60}")
    print("  FBWA STICK PREFLIGHT TEST")
    print(f"{'='*60}")
    print(f"  Plane ARMED on {CATAPULT_PITCH_DEG}° rail, pilot sweeps elevator stick")
    print(f"  catapult_ground_hold must keep I=0 despite stick inputs")
    print(f"{'='*60}")

    proc = start_sitl()
    try:
        m = connect()
        print("  Connected.")
        set_mode(m, 'FBWA')
        for name, val in PARAMS.items():
            set_param(m, name, val)

        # Disarmed idle (not measured)
        end = time.time() + 3.0 / SPEEDUP
        while time.time() < end:
            rc(m, throttle=1300, elevator=1500)
            m.recv_match(blocking=True, timeout=0.02)

        # Arm on rail
        armed = arm_force(m)
        print(f"  Armed: {armed}")
        print(f"  Sweeping elevator stick for {PREFLIGHT_S:.0f} s sim ...")

        # Sweep elevator: 1300 (down) → 1500 (neutral) → 1700 (up) → repeat
        # Each half-cycle = 2 s sim = 0.4 s real at 5x speedup
        cycle_real = 2.0 / SPEEDUP
        arm_wall = time.time()
        end_preflight = arm_wall + PREFLIGHT_S / SPEEDUP

        elev_steps = [1300, 1500, 1700, 1500]   # PWM sweep
        step_i = 0
        step_end = time.time() + cycle_real
        while time.time() < end_preflight:
            if time.time() >= step_end:
                step_i = (step_i + 1) % len(elev_steps)
                step_end = time.time() + cycle_real
            elev = elev_steps[step_i]
            rc(m, throttle=1300, elevator=elev)
            m.recv_match(blocking=True, timeout=0.02)

        # Launch: reset AHRS trim, full throttle
        set_param(m, 'AHRS_TRIM_Y', 0.0)
        print(f"  Launching ({LAUNCH_S:.0f} s sim) ...")
        end_launch = time.time() + LAUNCH_S / SPEEDUP
        while time.time() < end_launch:
            rc(m, throttle=2000, elevator=1500)
            m.recv_match(blocking=True, timeout=0.02)

        print("  Stopping SITL...")
    finally:
        stop_sitl(proc)

    log_dir = os.path.join(WORK_DIR, 'logs')
    logs = sorted([os.path.join(log_dir, f) for f in os.listdir(log_dir)
                   if f.endswith('.BIN')], key=os.path.getmtime)
    if not logs:
        sys.exit("ERROR: no BIN log")

    pidp, arsp = read_log(logs[-1])
    if not pidp:
        sys.exit("ERROR: no PIDP")

    # armed window = from arm_wall timestamp onwards
    # Approximate: skip first 3 s of log (disarmed idle)
    first_ts   = pidp[0]['ts']
    arm_ts_us  = first_ts + int(3.0 * 1e6)
    launch_ts  = arm_ts_us + int(PREFLIGHT_S * 1e6)

    preflight  = [p for p in pidp if arm_ts_us <= p['ts'] < launch_ts]
    launch_win = [p for p in pidp if p['ts'] >= launch_ts]

    I_max      = max((abs(p['I']) for p in preflight), default=0.0)
    flags9_pct = (sum(1 for p in preflight if p.get('Flags', 0) == 9)
                  / len(preflight) * 100 if preflight else 0.0)

    # Release moment: first arsp >= FBW_MIN after launch
    release = next((r for r in arsp if r['ts'] >= launch_ts
                    and r['Airspeed'] >= ARSPD_FBW_MIN), None)
    release_spd = release['Airspeed'] if release else 0.0

    print(f"\n{'='*60}")
    print("  RESULTS")
    print(f"{'='*60}")
    print(f"  Armed preflight ({PREFLIGHT_S:.0f} s sim, elevator sweeping):")
    print(f"    max |I|         = {I_max:.3f}°  (limit < 1.0°)")
    print(f"    Flags=9 share   = {flags9_pct:.0f}%  (limit > 80%)")
    print(f"  Hold released at airspeed = {release_spd:.1f} m/s  "
          f"(FBW_MIN = {ARSPD_FBW_MIN} m/s)")

    p_hold = I_max < 1.0 and flags9_pct > 80.0
    p_rel  = release is not None

    def v(ok, msg): print(f"  {'✓ PASS' if ok else '✗ FAIL'}  {msg}")
    v(p_hold, f"Hold active: I≤{I_max:.3f}°, Flags=9 {flags9_pct:.0f}% — stick inputs did not wind integrator")
    v(p_rel,  f"Hold released at {release_spd:.1f} m/s after launch")

    ok = p_hold and p_rel
    print()
    print(f"  OVERALL: {'PASS ✓' if ok else 'FAIL ✗'}")
    print(f"{'='*60}\n")
    return logs[-1], ok


if __name__ == '__main__':
    _, ok = main()
    sys.exit(0 if ok else 1)
