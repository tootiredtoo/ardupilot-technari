#!/usr/bin/env python3
"""
SITL test — headwind on catapult rail, no premature hold release.

Scenario: plane sits armed on the 12.35° rail with a steady headwind.
Wind speed is varied: 3 m/s, 6 m/s, 9 m/s — all BELOW ARSPD_FBW_MIN.
catapult_ground_hold must NOT release due to wind alone; I must stay 0.
Then a real launch raises airspeed above FBW_MIN → hold releases correctly.

Pass criteria per wind level:
  - max |I| while armed with headwind < 1°  (hold still active)
  - Flags=9 > 80% during hold
After launch:
  - hold releases when airspeed >= FBW_MIN
  - no nose-dive (min pitch > -10° in 5 s post-launch)
"""

import math, os, shutil, signal, subprocess, sys, time
from pymavlink import mavutil

WORK_DIR = '/tmp/sitl_headwind_rail'
PORT     = 5778
SPEEDUP  = 5

CATAPULT_PITCH_DEG = 12.35
AHRS_TRIM_Y_RAIL   = -math.radians(CATAPULT_PITCH_DEG)
ARSPD_FBW_MIN      = 12.0

# Wind levels to test (all must be < ARSPD_FBW_MIN)
WIND_LEVELS = [3.0, 6.0, 9.0]   # m/s
HOLD_S      = 6.0   # simulated seconds at each wind level (armed)
LAUNCH_S    = 5.0   # post-launch window

PARAMS = {
    'PTCH_RATE_I':      0.15,
    'PTCH_RATE_IMAX':   0.666,
    'PTCH_RATE_FF':     0.345,
    'LOG_DISARMED':     1,
    'SIM_WIND_SPD':     0.0,
    'SIM_WIND_TURB':    0.0,
    'SIM_WIND_DIR':     0.0,   # headwind (plane faces 0°)
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
    raise RuntimeError("Failed to connect")


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
    pidp, arsp, att = [], [], []
    while True:
        msg = mlog.recv_match(type=['PIDP', 'ARSP', 'ATT'], blocking=False)
        if msg is None:
            break
        ts = msg.TimeUS
        t  = msg.get_type()
        if t == 'PIDP':
            pidp.append({'ts': ts, 'I': msg.I,
                         'Flags': getattr(msg, 'Flags', 0)})
        elif t == 'ARSP':
            arsp.append({'ts': ts, 'Airspeed': msg.Airspeed})
        elif t == 'ATT':
            att.append({'ts': ts, 'Pitch': msg.Pitch})
    return pidp, arsp, att


def main():
    print(f"\n{'='*60}")
    print("  HEADWIND ON RAIL TEST")
    print(f"{'='*60}")
    for w in WIND_LEVELS:
        print(f"  Wind {w:.0f} m/s headwind — hold must NOT release (< FBW_MIN {ARSPD_FBW_MIN} m/s)")
    print(f"  Then real launch → hold releases at >= {ARSPD_FBW_MIN} m/s")
    print(f"{'='*60}")

    # Timeline markers (simulated seconds from first PIDP sample)
    windows = []   # list of (label, wind, t_start_sim, t_end_sim)
    t_sim = 2.0    # small idle first
    t_sim += 2.0   # warm-up armed window before first wind

    proc = start_sitl()
    try:
        m = connect()
        print("  Connected.")
        set_mode(m, 'FBWA')
        for name, val in PARAMS.items():
            set_param(m, name, val)

        # Brief disarmed idle
        drain(m, 2.0 / SPEEDUP, throttle=1300)

        arm_force(m)
        print(f"  Armed.")

        # Warm-up: calm, 2 s sim
        drain(m, 2.0 / SPEEDUP, throttle=1300)
        t_sim_cur = 4.0   # idle(2) + warmup(2)

        for wind in WIND_LEVELS:
            set_param(m, 'SIM_WIND_SPD', wind)
            set_param(m, 'SIM_WIND_DIR', 0.0)   # direct headwind
            t_start = t_sim_cur
            print(f"  Wind = {wind:.0f} m/s  ({HOLD_S:.0f} s sim) ...")
            drain(m, HOLD_S / SPEEDUP, throttle=1300)
            t_sim_cur += HOLD_S
            windows.append((f"wind {wind:.0f} m/s", wind, t_start, t_sim_cur))

        # Stop wind, launch
        set_param(m, 'SIM_WIND_SPD', 0.0)
        set_param(m, 'AHRS_TRIM_Y',  0.0)
        print(f"  LAUNCH ({LAUNCH_S:.0f} s sim) ...")
        t_launch_sim = t_sim_cur
        drain(m, LAUNCH_S / SPEEDUP, throttle=2000)

        print("  Stopping SITL...")
    finally:
        stop_sitl(proc)

    log_dir = os.path.join(WORK_DIR, 'logs')
    logs = sorted([os.path.join(log_dir, f) for f in os.listdir(log_dir)
                   if f.endswith('.BIN')], key=os.path.getmtime)
    if not logs:
        sys.exit("ERROR: no BIN log")

    pidp, arsp, att = read_log(logs[-1])
    if not pidp:
        sys.exit("ERROR: no PIDP")

    first_ts = pidp[0]['ts']

    print(f"\n{'='*60}")
    print("  RESULTS")
    print(f"{'='*60}")

    all_pass = True
    for label, wind, t0s, t1s in windows:
        t0_us = first_ts + int(t0s * 1e6)
        t1_us = first_ts + int(t1s * 1e6)
        w = [p for p in pidp if t0_us <= p['ts'] < t1_us]
        if not w:
            print(f"  WARNING: no samples for {label}")
            continue
        I_max  = max(abs(p['I']) for p in w)
        f9_pct = sum(1 for p in w if p.get('Flags', 0) == 9) / len(w) * 100
        ok     = I_max < 1.0 and f9_pct > 80.0
        all_pass = all_pass and ok
        print(f"  {'✓' if ok else '✗'}  {label:12s}  "
              f"max|I|={I_max:.3f}°  Flags=9 {f9_pct:.0f}%  "
              f"{'hold OK' if ok else 'HOLD FAILED'}")

    # Post-launch check
    launch_ts_us = first_ts + int(t_launch_sim * 1e6)
    release = next((r for r in arsp
                    if r['ts'] >= launch_ts_us and r['Airspeed'] >= ARSPD_FBW_MIN), None)
    rel_spd = release['Airspeed'] if release else 0.0
    p_rel = release is not None

    att_post = [a for a in att
                if launch_ts_us <= a['ts'] < launch_ts_us + int(LAUNCH_S * 1e6)]
    min_pitch = min((a['Pitch'] for a in att_post), default=0.0)
    p_dive = min_pitch > -10.0
    all_pass = all_pass and p_rel and p_dive

    print(f"  {'✓' if p_rel else '✗'}  Release at launch: {rel_spd:.1f} m/s "
          f"(FBW_MIN={ARSPD_FBW_MIN})")
    print(f"  {'✓' if p_dive else '✗'}  No dive post-launch: min pitch = {min_pitch:+.2f}°")

    print()
    print(f"  OVERALL: {'PASS ✓' if all_pass else 'FAIL ✗'}")
    print(f"{'='*60}\n")
    return logs[-1], all_pass


if __name__ == '__main__':
    _, ok = main()
    sys.exit(0 if ok else 1)
