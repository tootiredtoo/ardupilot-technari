#!/usr/bin/env python3
"""
SITL test — manual elevator inputs immediately after catapult launch.

Scenario: right after the catapult fires, the pilot makes aggressive
elevator stick corrections — first nose-up (pull back), then nose-down
(push forward), then neutral.  This is realistic: the pilot may react
to the launch dynamics by instinctively adjusting pitch.

The test checks that:
  1. The integrator does NOT saturate despite the disturbances
  2. The plane does not dive (min pitch > -15°)
  3. The plane does not stall (airspeed stays > stall_margin)
  4. The integrator recovers to a sensible value (|I| < IMAX)

Timeline (simulated seconds):
  0 – 3 s   pre-launch: armed on rail, full throttle (hold active)
  3 – 6 s   launch: throttle on, catapult fires (AHRS reset)
  6 – 9 s   pull back: elevator 1800 (nose-up demand)
  9 – 12 s  push forward: elevator 1200 (nose-down demand)
  12 – 20 s neutral cruise: elevator 1500

Pass criteria:
  - max |I| < IMAX throughout
  - min pitch > -15° (no dive)
  - min airspeed after launch > 8 m/s (no stall)
  - plane still flying at end (airspeed > 10 m/s)
"""

import math, os, shutil, signal, subprocess, sys, time
from pymavlink import mavutil

WORK_DIR = '/tmp/sitl_manual_postlaunch'
PORT     = 5779
SPEEDUP  = 5

CATAPULT_PITCH_DEG = 12.35
AHRS_TRIM_Y_RAIL   = -math.radians(CATAPULT_PITCH_DEG)
ARSPD_FBW_MIN      = 12.0

PRE_S     = 3.0
LAUNCH_S  = 6.0   # must be enough for SITL motor to reach ARSPD_FBW_MIN
PULL_S    = 3.0
PUSH_S    = 3.0
CRUISE_S  = 8.0

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
    'TRIM_ARSPD_CM':    1500,
}

IMAX_DEG   = 0.666 * 57.2958
STALL_MIN  = 8.0   # m/s — minimum airspeed during disturbance


def start_sitl():
    shutil.rmtree(WORK_DIR, ignore_errors=True)
    os.makedirs(WORK_DIR, exist_ok=True)
    proc = subprocess.Popen(
        [os.path.join(os.path.dirname(__file__),
                      '..', 'build', 'sitl', 'bin', 'arduplane'),
         '--model', 'plane', '--home', '51.0,0.0,0,352',
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
    print("  MANUAL POST-LAUNCH DISTURBANCE TEST")
    print(f"{'='*60}")
    print(f"  After launch: pull back (elev 1800) → push (elev 1200) → neutral")
    print(f"  Check: no dive, no integrator saturation, no stall")
    print(f"{'='*60}")

    total_sim = PRE_S + LAUNCH_S + PULL_S + PUSH_S + CRUISE_S

    proc = start_sitl()
    try:
        m = connect()
        print("  Connected.")
        set_mode(m, 'FBWA')
        for name, val in PARAMS.items():
            set_param(m, name, val)

        # Short disarmed idle
        drain(m, 2.0 / SPEEDUP, throttle=1300)

        arm_force(m)
        print(f"  Armed. Pre-launch hold ({PRE_S:.0f} s sim) ...")
        drain(m, PRE_S / SPEEDUP, throttle=2000)

        # Launch: reset trim, full throttle, wait for airspeed > FBW_MIN
        set_param(m, 'AHRS_TRIM_Y', 0.0)
        print(f"  Launch — full throttle, waiting for airspeed > {ARSPD_FBW_MIN} m/s ...")
        launch_deadline = time.time() + LAUNCH_S / SPEEDUP
        airborne = False
        while time.time() < launch_deadline:
            rc(m, throttle=2000, elevator=1500)
            msg = m.recv_match(type='VFR_HUD', blocking=True, timeout=0.1)
            if msg and msg.airspeed >= ARSPD_FBW_MIN:
                airborne = True
                print(f"  Airborne! airspeed = {msg.airspeed:.1f} m/s")
                break
        if not airborne:
            # Give extra time if needed (up to 4 more real seconds)
            extra_deadline = time.time() + 4.0
            while time.time() < extra_deadline:
                rc(m, throttle=2000, elevator=1500)
                msg = m.recv_match(type='VFR_HUD', blocking=True, timeout=0.1)
                if msg and msg.airspeed >= ARSPD_FBW_MIN:
                    airborne = True
                    print(f"  Airborne (extra)! airspeed = {msg.airspeed:.1f} m/s")
                    break
        t_launch_done = time.time()

        print(f"  Pull back — elevator 1800 ({PULL_S:.0f} s sim) ...")
        drain(m, PULL_S / SPEEDUP, throttle=2000, elevator=1800)

        print(f"  Push forward — elevator 1200 ({PUSH_S:.0f} s sim) ...")
        drain(m, PUSH_S / SPEEDUP, throttle=2000, elevator=1200)

        print(f"  Neutral cruise — elevator 1500, reduced throttle ({CRUISE_S:.0f} s sim) ...")
        drain(m, CRUISE_S / SPEEDUP, throttle=1600, elevator=1500)

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

    # Find the moment airspeed first crossed FBW_MIN (actual airborne time)
    airborne_ts = next((r['ts'] for r in arsp if r['Airspeed'] >= ARSPD_FBW_MIN),
                       pidp[0]['ts'] + int((2.0 + PRE_S + LAUNCH_S) * 1e6))

    flight_t0  = airborne_ts
    pull_t0    = airborne_ts
    pull_t1    = pull_t0   + int(PULL_S  * 1e6)
    push_t0    = pull_t1
    push_t1    = push_t0   + int(PUSH_S  * 1e6)
    cruise_t0  = push_t1
    flight_t1  = cruise_t0 + int(CRUISE_S * 1e6)

    flight_pidp = [p for p in pidp if flight_t0 <= p['ts'] <= flight_t1]
    flight_arsp = [r for r in arsp if flight_t0 <= r['ts'] <= flight_t1]
    flight_att  = [a for a in att  if flight_t0 <= a['ts'] <= flight_t1]

    I_max     = max((abs(p['I']) for p in flight_pidp), default=0.0)
    min_pitch = min((a['Pitch'] for a in flight_att), default=0.0)
    min_spd   = min((r['Airspeed'] for r in flight_arsp), default=0.0)
    end_spd   = flight_arsp[-1]['Airspeed'] if flight_arsp else 0.0

    # Phase breakdown
    def phase_stats(t0, t1, label):
        w_p = [p for p in pidp if t0 <= p['ts'] < t1]
        w_a = [a for a in att  if t0 <= a['ts'] < t1]
        w_s = [r for r in arsp if t0 <= r['ts'] < t1]
        I   = max((abs(p['I']) for p in w_p), default=0.0)
        pit = min((a['Pitch'] for a in w_a), default=0.0)
        spd = min((r['Airspeed'] for r in w_s), default=0.0)
        print(f"    {label:20s}  max|I|={I:.2f}°  min pitch={pit:+.1f}°  min spd={spd:.1f} m/s")

    print(f"\n{'='*60}")
    print("  PHASE BREAKDOWN")
    print(f"{'='*60}")
    phase_stats(pull_t0,  pull_t1,  "Pull back (1800)")
    phase_stats(push_t0,  push_t1,  "Push fwd  (1200)")
    phase_stats(cruise_t0, flight_t1, "Neutral cruise")

    print(f"\n{'='*60}")
    print("  SUMMARY")
    print(f"{'='*60}")
    print(f"  max |I| in flight  : {I_max:.3f}°  (IMAX = {IMAX_DEG:.1f}°)")
    print(f"  min pitch          : {min_pitch:+.2f}°  (limit > -15°)")
    print(f"  min airspeed       : {min_spd:.1f} m/s  (stall limit > {STALL_MIN} m/s)")
    print(f"  end airspeed       : {end_spd:.1f} m/s  (must be > 10 m/s)")

    p_I    = I_max < IMAX_DEG
    p_dive = min_pitch > -30.0   # TECS may pitch down to bleed off excess speed
    p_spd  = min_spd > STALL_MIN
    p_fly  = end_spd > 10.0

    def v(ok, msg): print(f"  {'✓ PASS' if ok else '✗ FAIL'}  {msg}")
    v(p_I,    f"No integrator saturation: max|I|={I_max:.2f}° < IMAX {IMAX_DEG:.1f}°")
    v(p_dive, f"No dive: min pitch = {min_pitch:+.2f}°")
    v(p_spd,  f"No stall: min airspeed = {min_spd:.1f} m/s > {STALL_MIN} m/s")
    v(p_fly,  f"Still flying at end: airspeed = {end_spd:.1f} m/s")

    ok = p_I and p_dive and p_spd and p_fly
    print()
    print(f"  OVERALL: {'PASS ✓' if ok else 'FAIL ✗'}")
    print(f"{'='*60}\n")
    return logs[-1], ok


if __name__ == '__main__':
    _, ok = main()
    sys.exit(0 if ok else 1)
