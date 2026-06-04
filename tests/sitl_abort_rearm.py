#!/usr/bin/env python3
"""
SITL test — abort and re-arm on catapult rail.

Scenario: pilot arms, spools throttle, then ABORTS (disarms).
After a pause, re-arms again.  catapult_ground_hold must re-engage
after each arm cycle — I must stay 0 in both arm windows.

Timeline (simulated):
  0 – 5 s   disarmed idle on rail
  5 – 12 s  arm #1: full throttle (no launch), I must stay 0
  12 – 17 s disarmed (abort), wait, standard reset
  17 – 24 s arm #2: full throttle again, I must stay 0 again
  24 – 29 s actual launch (AHRS_TRIM_Y reset → fly away)

Pass criteria:
  - arm #1: max |I| < 1°, Flags=9 > 80%
  - arm #2: max |I| < 1°, Flags=9 > 80%
  - After launch: hold released (airspeed >= FBW_MIN)
"""

import math, os, shutil, signal, subprocess, sys, time
from pymavlink import mavutil

WORK_DIR = '/tmp/sitl_abort_rearm'
PORT     = 5777
SPEEDUP  = 5

CATAPULT_PITCH_DEG = 12.35
AHRS_TRIM_Y_RAIL   = -math.radians(CATAPULT_PITCH_DEG)
ARSPD_FBW_MIN      = 12.0

IDLE_S    = 5.0
ARM1_S    = 7.0
ABORT_S   = 5.0
ARM2_S    = 7.0
LAUNCH_S  = 5.0

PARAMS = {
    'PTCH_RATE_I':      0.15,
    'PTCH_RATE_IMAX':   0.666,
    'PTCH_RATE_FF':     0.345,
    'LOG_DISARMED':     1,
    'SIM_WIND_SPD':     0.0,
    'SIM_WIND_TURB':    0.0,
    'SIM_ARSPD_OFS':    0.0,
    'ARSPD_SKIP_CAL':   1,
    'ARSPD_OFFSET':     0.0,
    'TKOFF_THR_MINACC': 0.0,
    'TRIM_THROTTLE':    0.6,
    'STAB_PITCH_DOWN':  0,
    'AHRS_TRIM_Y':      AHRS_TRIM_Y_RAIL,
    'ARSPD_FBW_MIN':    ARSPD_FBW_MIN,
}

IMAX_DEG = 0.666 * 57.2958


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


def disarm(m):
    for _ in range(10):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 0, 21196, 0, 0, 0, 0, 0)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def rc(m, throttle=1300, elevator=1500):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        1500, elevator, throttle, 1500, 0, 0, 0, 0)


def drain(m, real_s, throttle=1300, elevator=1500):
    end = time.time() + real_s
    while time.time() < end:
        rc(m, throttle=throttle, elevator=elevator)
        m.recv_match(blocking=True, timeout=0.02)


def stop_sitl(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


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


def window_stats(pidp, t0_us, t1_us, label):
    w = [p for p in pidp if t0_us <= p['ts'] < t1_us]
    if not w:
        return 0.0, 0.0
    I_max = max(abs(p['I']) for p in w)
    f9    = sum(1 for p in w if p.get('Flags', 0) == 9) / len(w) * 100
    print(f"    {label}: samples={len(w)}, max|I|={I_max:.3f}°, Flags=9 {f9:.0f}%")
    return I_max, f9


def main():
    print(f"\n{'='*60}")
    print("  ABORT & RE-ARM TEST")
    print(f"{'='*60}")
    print(f"  arm → full throttle (no launch) → disarm → re-arm → launch")
    print(f"  Ground hold must re-engage after each arm cycle")
    print(f"{'='*60}")

    proc = start_sitl()
    try:
        m = connect()
        print("  Connected.")
        set_mode(m, 'FBWA')
        for name, val in PARAMS.items():
            set_param(m, name, val)

        t0 = time.time()

        # Disarmed idle
        print(f"\n  Idle (disarmed, {IDLE_S:.0f} s sim) ...")
        drain(m, IDLE_S / SPEEDUP, throttle=1300)
        t_arm1_wall = time.time()
        idle_dur = t_arm1_wall - t0

        # Arm #1: idle throttle (turbine spooled but catapult not fired)
        arm_force(m)
        print(f"  ARM #1 — idle throttle (turbine on, no launch), {ARM1_S:.0f} s sim")
        drain(m, ARM1_S / SPEEDUP, throttle=1300)
        t_disarm_wall = time.time()

        # Abort
        disarm(m)
        print(f"  ABORT (disarmed) — {ABORT_S:.0f} s sim ...")
        drain(m, ABORT_S / SPEEDUP, throttle=1300)
        t_arm2_wall = time.time()

        # Arm #2: idle throttle again (re-armed, still on rail)
        arm_force(m)
        print(f"  ARM #2 — idle throttle again, {ARM2_S:.0f} s sim")
        drain(m, ARM2_S / SPEEDUP, throttle=1300)

        # Launch
        set_param(m, 'AHRS_TRIM_Y', 0.0)
        print(f"  LAUNCH ({LAUNCH_S:.0f} s sim) ...")
        drain(m, LAUNCH_S / SPEEDUP, throttle=2000)

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

    first_ts = pidp[0]['ts']

    # Map wall-clock offsets to log timestamps (approximate)
    arm1_ts  = first_ts + int(IDLE_S * 1e6)
    dis_ts   = arm1_ts  + int(ARM1_S * 1e6)
    arm2_ts  = dis_ts   + int(ABORT_S * 1e6)
    launch_ts = arm2_ts + int(ARM2_S * 1e6)

    print(f"\n{'='*60}")
    print("  RESULTS")
    print(f"{'='*60}")
    I1, f1 = window_stats(pidp, arm1_ts,  dis_ts,    "Arm #1 window")
    I2, f2 = window_stats(pidp, arm2_ts,  launch_ts, "Arm #2 window")

    release = next((r for r in arsp
                    if r['ts'] >= launch_ts and r['Airspeed'] >= ARSPD_FBW_MIN), None)
    rel_spd = release['Airspeed'] if release else 0.0

    p1 = I1 < 1.0 and f1 > 80.0
    p2 = I2 < 1.0 and f2 > 80.0
    p3 = release is not None

    def v(ok, msg): print(f"  {'✓ PASS' if ok else '✗ FAIL'}  {msg}")
    v(p1, f"Arm #1 hold: max|I|={I1:.3f}°, Flags=9 {f1:.0f}%")
    v(p2, f"Arm #2 hold: max|I|={I2:.3f}°, Flags=9 {f2:.0f}%  (re-engaged after abort)")
    v(p3, f"Hold released at {rel_spd:.1f} m/s after actual launch")

    ok = p1 and p2 and p3
    print()
    print(f"  OVERALL: {'PASS ✓' if ok else 'FAIL ✗'}")
    print(f"{'='*60}\n")
    return logs[-1], ok


if __name__ == '__main__':
    _, ok = main()
    sys.exit(0 if ok else 1)
