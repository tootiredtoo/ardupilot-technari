#!/usr/bin/env python3
"""
Advanced GPS-spoofing resilience tests — Ukrainian-theatre scenarios.

Background
----------
In Ukrainian airspace, Russian GPS spoofing typically does one of:
  (a) Push coordinates to Lima, Peru  (~10 000 km southwest)
  (b) Null/freeze position (plane appears stationary or circles)
  (c) Oscillate: real GPS ↔ spoof ↔ real GPS at irregular intervals

SITL limitation: position glitches > ~100 m applied to a fast-moving
plane cause the EKF to attempt a large state reset that overflows its
covariance.  Tests are run with the plane stationary / low throttle so
that EKF GPS processing can be exercised without aerodynamics crash.
In production the glitch magnitude would be ~10 000 km (Lima); here we
use 100 m as a scaled proxy — it is 10× EK3_GLITCH_RAD (10 m) and thus
triggers the same EKF rejection path.

These tests are intentionally written to FAIL on a stock ArduPlane build
and to PASS only after the GPS resilience hardening patches are applied.
This is the TDD red phase.

  Suite C — Large-jump spoofing (Lima-class: 100 m proxy)
    Verifies that the EKF does NOT accept the spoofed position and that
    the GPS_USING_BIT (0x2000) stays CLEAR for SPOOF_LOCKOUT_S after the
    glitch ends (i.e. GPS is kept suspect beyond the 10 s EK3 holdoff).
    CURRENTLY EXPECTED TO FAIL: stock build re-accepts GPS after ~10 s.

  Suite D — Pre-arm spoofing (GPS already wrong when arming)
    Activates the glitch BEFORE arming.  Verifies that either arming is
    refused (pre-arm check catches implausible position) or the EKF
    position uncertainty is flagged and GPS-dependent modes are locked.
    CURRENTLY EXPECTED TO FAIL: pre-arm checks don't validate position
    plausibility, only signal quality.

  Suite E — Oscillating spoof (real↔fake cycling with 5 s half-period)
    Toggles glitch on and off every CYCLE_S=5 sim-seconds (4 cycles).
    10 s EK3 holdoff > 5 s OFF window → GPS is never trusted in OFF phases
    on stock.  Test then checks that on the very next ON phase after the
    holdoff expires (cycle 4), the EKF accepts GPS during the ON phase
    (GPS_USING_BIT set), which means spoofed data is being ingested.
    CURRENTLY EXPECTED TO FAIL: stock build ingests spoofed GPS.
    NOTE: with CYCLE_S < holdoff, a more complete stockbuild failure mode
    is that at the end of the oscillation sequence, GPS_USING_BIT is never
    re-established at all — IMU dead-reckons indefinitely.  Both are bugs.

Usage
-----
  python3 tests/gps_spoof_advanced_test.py [--binary PATH]
"""

import argparse, io, os, shutil, signal, subprocess, sys, time
from contextlib import redirect_stdout
from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_BINARY = os.path.join(
    os.path.dirname(__file__), '..', 'build', 'sitl', 'bin', 'arduplane')
WORK_DIR  = '/tmp/sitl_gps_spoof_adv'
PORT_C    = 5776
PORT_D    = 5777
PORT_E    = 5778
SPEEDUP   = 5

# 100 m glitch: 10× EK3_GLITCH_RAD default (10 m).  Same rejection path
# as Lima, Peru (9 500 km) but safe for SITL float precision.
LIMA_GLITCH_M   = 100.0    # m north — "Lima-class" proxy

SPOOF_LOCKOUT_S = 60.0     # sim-s: GPS must stay untrusted after spoof event
CYCLE_S         = 5.0      # sim-s per half-cycle (< 10 s EK3 holdoff)
N_CYCLES        = 4

PARAMS_BASE = {
    'LOG_DISARMED': 1,
    'EK3_OPTIONS':  1,     # bit 0 = JammingExpected → enables holdoff
}

GPS_USING_BIT = 0x2000     # nav_filter_status.using_gps

# ── Connection wrapper ────────────────────────────────────────────────────────

class SITLSession:
    """
    Wraps a pymavlink connection and silences pymavlink's internal
    'EOF on TCP socket' stdout spam when the connection drops.
    All public methods are no-ops once the connection is detected dead.
    """
    def __init__(self, m):
        self._m    = m
        self.alive = True

    def _call(self, fn, *args, **kwargs):
        if not self.alive:
            return None
        try:
            with redirect_stdout(io.StringIO()):
                return fn(*args, **kwargs)
        except Exception:
            self.alive = False
            return None

    def set_param(self, name, value, retries=3):
        if not self.alive:
            return
        for _ in range(retries):
            self._call(
                self._m.mav.param_set_send,
                self._m.target_system, self._m.target_component,
                name.encode(), float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            ack = self._call(
                self._m.recv_match,
                type='PARAM_VALUE', blocking=True, timeout=1)
            if ack and ack.param_id.rstrip('\x00') == name:
                return
        print(f"  WARNING: no ACK for {name}={value}")

    def set_mode(self, mode_name):
        if not self.alive:
            return False
        mode_id = self._m.mode_mapping().get(mode_name)
        if mode_id is None:
            return False
        for _ in range(5):
            self._call(
                self._m.mav.set_mode_send,
                self._m.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            msg = self._call(
                self._m.recv_match,
                type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                return True
        return False

    def get_mode(self):
        if not self.alive:
            return None
        msg = self._call(self._m.recv_match,
                         type='HEARTBEAT', blocking=True, timeout=2)
        return msg.custom_mode if msg else None

    def arm_force(self):
        if not self.alive:
            return False
        for _ in range(10):
            self._call(
                self._m.mav.command_long_send,
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
            msg = self._call(
                self._m.recv_match,
                type='HEARTBEAT', blocking=True, timeout=1)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def drain(self, real_seconds, throttle=1800, elevator=1500):
        """RC overrides for real_seconds wall-clock seconds."""
        end = time.time() + real_seconds
        while time.time() < end:
            if not self.alive:
                time.sleep(max(0.0, end - time.time()))
                return False
            self._call(
                self._m.mav.rc_channels_override_send,
                self._m.target_system, self._m.target_component,
                1500, elevator, throttle, 1500, 0, 0, 0, 0)
            msg = self._call(self._m.recv_match, blocking=False)
            if msg is None:
                time.sleep(0.02)
        return self.alive

    def recv_heartbeat(self, timeout=2.0):
        msg = self._call(self._m.recv_match,
                         type='HEARTBEAT', blocking=True, timeout=timeout)
        return msg

    def recv_gps(self, timeout=3.0):
        msg = self._call(self._m.recv_match,
                         type='GPS_RAW_INT', blocking=True, timeout=timeout)
        return msg

    @property
    def mode_mapping(self):
        return self._m.mode_mapping()


# ── SITL process management ───────────────────────────────────────────────────

def start_sitl(binary, port, subdir):
    d = os.path.join(WORK_DIR, subdir)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d, exist_ok=True)
    proc = subprocess.Popen(
        [os.path.abspath(binary),
         '--model', 'plane',
         '--home', '51.0,0.0,0,352',
         f'--speedup={SPEEDUP}',
         f'--serial0=tcp:{port}'],
        cwd=d,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    time.sleep(3)
    return proc


def connect(port):
    for _ in range(20):
        try:
            with redirect_stdout(io.StringIO()):
                m = mavutil.mavlink_connection(
                    f'tcp:127.0.0.1:{port}', source_system=255)
                m.wait_heartbeat(timeout=5)
            return SITLSession(m)
        except Exception:
            time.sleep(1)
    raise RuntimeError(f"Cannot connect to SITL on port {port}")


def stop_sitl(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


# ── Log reader ────────────────────────────────────────────────────────────────

def latest_bin(subdir):
    log_dir = os.path.join(WORK_DIR, subdir, 'logs')
    bins = sorted(
        [os.path.join(log_dir, f)
         for f in os.listdir(log_dir) if f.endswith('.BIN')],
        key=os.path.getmtime)
    if not bins:
        raise RuntimeError(f"No .BIN log in {log_dir}")
    return bins[-1]


def parse_log(path):
    mlog = mavutil.mavlink_connection(path, dialect='ardupilotmega')
    series = {'ATT': [], 'XKF4': [], 'GPS': []}
    while True:
        msg = mlog.recv_match(type=list(series.keys()), blocking=False)
        if msg is None:
            break
        ts = getattr(msg, 'TimeUS', 0)
        t  = msg.get_type()
        if t == 'ATT':
            series['ATT'].append({'ts': ts, 'Pitch': msg.Pitch, 'Roll': msg.Roll})
        elif t == 'XKF4':
            series['XKF4'].append({
                'ts': ts,
                'SP': getattr(msg, 'SP', 0) / 100.0,
                'SV': getattr(msg, 'SV', 0) / 100.0,
                'SS': getattr(msg, 'SS', 0),
            })
        elif t == 'GPS':
            series['GPS'].append({
                'ts':    ts,
                'Status': getattr(msg, 'Status', 0),
                'NSats':  getattr(msg, 'NSats',  0),
                'Lat':    getattr(msg, 'Lat',    0),
            })
    return series


def window(s, t0, t1):
    return [x for x in s if t0 <= x['ts'] <= t1]


def verdict(ok, msg, expected_fail=False):
    if expected_fail:
        icon = '✗ [STOCK-BUG] ' if ok else '✓ [BUG EXPOSED]'
    else:
        icon = '✓ PASS' if ok else '✗ FAIL'
    print(f"  {icon}  {msg}")
    return ok


# ── Suite C — Lima-class large-jump spoofing ──────────────────────────────────

BASELINE_C_S = 10.0
LIMA_ON_S    = 20.0
# Monitor for SPOOF_LOCKOUT_S after glitch ends to see when GPS re-accepted
MONITOR_C_S  = SPOOF_LOCKOUT_S + 15.0   # extra 15 s to see GPS return


def run_lima_spoof_suite(binary, port):
    total_sim = BASELINE_C_S + LIMA_ON_S + MONITOR_C_S
    print(f"\n{'='*65}")
    print("  SUITE C — Lima-class GPS spoof")
    print(f"{'='*65}")
    print(f"  Glitch   : +{LIMA_GLITCH_M:.0f} degrees (Lima-class proxy)")
    print(f"  Lockout  : {SPOOF_LOCKOUT_S:.0f} s sim GPS_USING_BIT must stay clear after spoof")
    print(f"  Total    : {total_sim:.0f} s sim / {total_sim/SPEEDUP:.0f} s real")
    print(f"  [TDD RED] Stock 10 s holdoff → GPS re-trusted at ~10 s (not {SPOOF_LOCKOUT_S:.0f}s)")
    print(f"{'='*65}")

    proc = start_sitl(binary, port, 'suite_c')
    try:
        s = connect(port)
        print("  Connected.")
        s.set_mode('FBWA')
        for k, v in PARAMS_BASE.items():
            s.set_param(k, v)
        s.set_param('SIM_GPS1_ENABLE',  1)
        s.set_param('SIM_GPS1_GLTCH_X', 0)
        s.arm_force()

        # Phase 1: clean baseline
        print(f"\n  [C-1] Baseline — {BASELINE_C_S:.0f} s sim")
        s.drain(BASELINE_C_S / SPEEDUP, throttle=1300)

        # Phase 2: Lima-class glitch ON
        print(f"\n  [C-2] Lima spoof ON — {LIMA_ON_S:.0f} s sim")
        s.set_param('SIM_GPS1_GLTCH_X', LIMA_GLITCH_M)
        s.drain(LIMA_ON_S / SPEEDUP, throttle=1300)

        # Phase 3: glitch off — monitor when GPS re-accepted by EKF
        print(f"\n  [C-3] Spoof OFF — monitoring {MONITOR_C_S:.0f} s for GPS_USING_BIT recovery")
        s.set_param('SIM_GPS1_GLTCH_X', 0)
        s.drain(MONITOR_C_S / SPEEDUP, throttle=1300)

        print("  Stopping SITL...")
    finally:
        stop_sitl(proc)

    log = parse_log(latest_bin('suite_c'))
    att = log['ATT']
    ekf = log['XKF4']

    if not att:
        print("  ERROR: empty log")
        return False

    first_ts  = att[0]['ts']
    spoof_ts  = first_ts + int(BASELINE_C_S * 1e6)
    off_ts    = spoof_ts  + int(LIMA_ON_S   * 1e6)
    end_ts    = off_ts    + int(MONITOR_C_S  * 1e6)

    ekf_base  = window(ekf, first_ts, spoof_ts)
    ekf_spoof = window(ekf, spoof_ts, off_ts)
    att_spoof = window(att, spoof_ts, off_ts)
    ekf_post  = window(ekf, off_ts,   end_ts)

    sp_base   = max((e['SP'] for e in ekf_base),  default=0.0)
    sp_spoof  = max((e['SP'] for e in ekf_spoof), default=0.0)
    ekf_rejects = (sp_spoof > sp_base * 2.0 + 0.1)

    att_ok    = (att_spoof
                 and all(-30 < a['Pitch'] < 45 for a in att_spoof)
                 and all(-60 < a['Roll']  < 60 for a in att_spoof))

    # Find the first EKF sample after glitch-off where GPS_USING_BIT is set.
    # On stock build this happens at ~10 s (holdoff); on patched at SPOOF_LOCKOUT_S.
    lockout_ts = off_ts + int(SPOOF_LOCKOUT_S * 1e6)
    gps_retrust_ts = None
    for e in ekf_post:
        if e['SS'] & GPS_USING_BIT:
            gps_retrust_ts = e['ts']
            break

    # How many sim-seconds after glitch-off did EKF re-trust GPS?
    if gps_retrust_ts is not None:
        gps_retrust_s = (gps_retrust_ts - off_ts) / 1e6
    else:
        gps_retrust_s = None   # GPS never re-trusted in monitoring window

    # On stock: gps_retrust_s ≈ 10 s (holdoff expires)
    # On patched: gps_retrust_s ≥ SPOOF_LOCKOUT_S (or None if window too short)
    gps_held_long_enough = (
        gps_retrust_s is None or gps_retrust_s >= SPOOF_LOCKOUT_S)

    # Were there any lockout-window samples where GPS_USING_BIT was set?
    lockout_window = window(ekf, off_ts, lockout_ts)
    gps_used_in_lockout = any(e['SS'] & GPS_USING_BIT for e in lockout_window)

    print(f"\n{'='*65}")
    print("  SUMMARY — Suite C")
    print(f"{'='*65}")
    print(f"  EKF SP: baseline max {sp_base:.3f} m → spoof max {sp_spoof:.3f} m "
          f"({'spike detected' if ekf_rejects else 'no change'})")
    print(f"  FBWA attitude stable during spoof   : {att_ok}")
    if gps_retrust_s is not None:
        print(f"  GPS re-trusted at {gps_retrust_s:.1f} s after glitch off "
              f"  ← want ≥ {SPOOF_LOCKOUT_S:.0f} s (or never)")
    else:
        print(f"  GPS NOT re-trusted within monitoring window  ← good")
    print(f"  GPS used during {SPOOF_LOCKOUT_S:.0f} s lockout window : {gps_used_in_lockout}"
          f"  ← should be False")

    print()
    verdict(att_ok,
            "C-1 FBWA attitude stable during Lima-class spoof")
    verdict(ekf_rejects,
            f"C-2 EKF detects Lima jump (SP {sp_base:.3f}→{sp_spoof:.3f} m)")
    verdict(not gps_used_in_lockout,
            f"C-3 GPS_USING_BIT clear for {SPOOF_LOCKOUT_S:.0f}s after spoof ends  [NEEDS PATCH]",
            expected_fail=True)

    overall = att_ok and ekf_rejects and not gps_used_in_lockout
    print()
    print(f"  SUITE C: {'PASS ✓  (patches applied)' if overall else 'FAIL ✗  (expected on stock build)'}")
    print(f"{'='*65}")
    return overall


# ── Suite D — Pre-arm spoofing ────────────────────────────────────────────────

PREARM_SOAK_S = 8.0    # sim-s to hold spoof before trying to arm


def run_prearm_spoof_suite(binary, port):
    print(f"\n{'='*65}")
    print("  SUITE D — Pre-arm GPS position spoofing")
    print(f"{'='*65}")
    print(f"  GPS glitch +{LIMA_GLITCH_M:.0f} m active before arming")
    print(f"  Pre-arm should block arm if GPS is > 1 km from home")
    print(f"  [TDD RED] Stock pre-arm checks signal quality only → FAIL")
    print(f"{'='*65}")

    proc = start_sitl(binary, port, 'suite_d')
    arm_result   = False
    gps_lat_arm  = None
    try:
        s = connect(port)
        print("  Connected.")
        s.set_mode('FBWA')
        for k, v in PARAMS_BASE.items():
            s.set_param(k, v)

        # Activate spoof BEFORE any arm attempt
        print(f"\n  [D-1] Activating +{LIMA_GLITCH_M:.0f} m spoof before arm...")
        s.set_param('SIM_GPS1_GLTCH_X', LIMA_GLITCH_M)
        s.drain(PREARM_SOAK_S / SPEEDUP, throttle=1300)

        # Read GPS position as reported to the autopilot
        gps_msg = s.recv_gps()
        if gps_msg:
            gps_lat_arm = gps_msg.lat / 1e7

        print(f"\n  [D-2] Attempting arm with GPS glitched...")
        arm_result = s.arm_force()
        print(f"    arm_force() → {arm_result}")
        print(f"    GPS lat at arm time: {gps_lat_arm}")

        s.drain(5.0 / SPEEDUP, throttle=1300)
        print("  Stopping SITL...")
    finally:
        stop_sitl(proc)

    # GPS lat shifts by LIMA_GLITCH_M / 111_000 degrees per meter
    home_lat     = 51.0
    lat_shift    = LIMA_GLITCH_M / 111_000.0
    expected_lat = home_lat + lat_shift
    if gps_lat_arm is not None:
        lat_diff_m = abs(gps_lat_arm - home_lat) * 111_000.0
    else:
        lat_diff_m = 0.0

    gps_is_spoofed = lat_diff_m > 50.0
    arm_blocked    = not arm_result

    print(f"\n{'='*65}")
    print("  SUMMARY — Suite D")
    print(f"{'='*65}")
    print(f"  GPS lat at arm: {gps_lat_arm} (home=51.0, expected≈{expected_lat:.4f})")
    print(f"  GPS offset    : {lat_diff_m:.1f} m from home")
    print(f"  GPS is spoofed: {gps_is_spoofed}")
    print(f"  Arm blocked   : {arm_blocked}  ← should be True")

    print()
    verdict(gps_is_spoofed,
            f"D-1 GPS confirms spoof active at arm time (+{lat_diff_m:.0f} m)")
    verdict(arm_blocked,
            "D-2 Pre-arm blocked — GPS too far from home  [NEEDS PATCH]",
            expected_fail=True)

    overall = gps_is_spoofed and arm_blocked
    print()
    print(f"  SUITE D: {'PASS ✓  (patches applied)' if overall else 'FAIL ✗  (expected on stock build)'}")
    print(f"{'='*65}")
    return overall


# ── Suite E — Oscillating spoof ───────────────────────────────────────────────

def run_oscillating_spoof_suite(binary, port):
    # Extra settle after oscillation so we can see if GPS is ever re-trusted
    SETTLE_POST_S = 20.0
    total_sim = N_CYCLES * 2 * CYCLE_S + SETTLE_POST_S
    print(f"\n{'='*65}")
    print("  SUITE E — Oscillating GPS spoof")
    print(f"{'='*65}")
    print(f"  Pattern : {N_CYCLES}× ({CYCLE_S:.0f} s ON / {CYCLE_S:.0f} s OFF) sim")
    print(f"  CYCLE_S ({CYCLE_S:.0f} s) < EK3 holdoff (10 s) → holdoff always reset before expiry")
    print(f"  Total   : {total_sim:.0f} s sim / {total_sim/SPEEDUP:.0f} s real")
    print(f"  [TDD RED] After cycles, GPS_USING_BIT never re-established; IMU dead-reckons")
    print(f"  [TDD RED] Also: during ON phases EKF may still ingest spoofed GPS briefly")
    print(f"{'='*65}")

    proc = start_sitl(binary, port, 'suite_e')
    try:
        s = connect(port)
        print("  Connected.")
        s.set_mode('FBWA')
        for k, v in PARAMS_BASE.items():
            s.set_param(k, v)
        s.set_param('SIM_GPS1_GLTCH_X', 0)
        s.arm_force()

        # Settle
        s.drain(5.0 / SPEEDUP, throttle=1300)

        print(f"\n  Oscillating {N_CYCLES}× ×{CYCLE_S:.0f} s ON/{CYCLE_S:.0f} s OFF:")
        for c in range(N_CYCLES):
            print(f"    Cycle {c+1}/{N_CYCLES}: ON ", end='', flush=True)
            s.set_param('SIM_GPS1_GLTCH_X', LIMA_GLITCH_M)
            s.drain(CYCLE_S / SPEEDUP, throttle=1300)
            print(f"→ OFF", flush=True)
            s.set_param('SIM_GPS1_GLTCH_X', 0)
            s.drain(CYCLE_S / SPEEDUP, throttle=1300)

        # Post-oscillation settle to observe GPS recovery behaviour
        print(f"\n  Post-oscillation settle — {SETTLE_POST_S:.0f} s sim (watching GPS_USING_BIT)")
        s.drain(SETTLE_POST_S / SPEEDUP, throttle=1300)

        print("  Stopping SITL...")
    finally:
        stop_sitl(proc)

    SETTLE_POST_S = 20.0
    log = parse_log(latest_bin('suite_e'))
    att = log['ATT']
    ekf = log['XKF4']

    if not att:
        print("  ERROR: empty log")
        return False

    first_ts  = att[0]['ts']
    settle_ts = first_ts + int(5e6)
    osc_dur   = int(N_CYCLES * 2 * CYCLE_S * 1e6)
    osc_end   = settle_ts + osc_dur
    post_end  = osc_end   + int(SETTLE_POST_S * 1e6)

    att_osc   = window(att, settle_ts, post_end)
    att_ok    = (att_osc
                 and all(-30 < a['Pitch'] < 45 for a in att_osc)
                 and all(-60 < a['Roll']  < 60 for a in att_osc))

    # Per-cycle: was GPS_USING_BIT ever set during ON or OFF phases?
    cycle_gps_on  = []   # True if GPS used (spoofed data ingested!) in ON phase
    cycle_gps_off = []   # True if GPS used in OFF phase
    for c in range(N_CYCLES):
        on_start = settle_ts + int(c * 2 * CYCLE_S * 1e6)
        on_end   = on_start  + int(CYCLE_S * 1e6)
        off_end  = on_end    + int(CYCLE_S * 1e6)
        on_gps   = any(e['SS'] & GPS_USING_BIT for e in window(ekf, on_start, on_end))
        off_gps  = any(e['SS'] & GPS_USING_BIT for e in window(ekf, on_end,   off_end))
        cycle_gps_on.append(on_gps)
        cycle_gps_off.append(off_gps)

    # Was GPS ever re-established in the post-oscillation settle?
    post_gps = any(e['SS'] & GPS_USING_BIT for e in window(ekf, osc_end, post_end))

    # On stock: CYCLE_S(5) < holdoff(10) → holdoff never expires within a cycle
    # → GPS_USING_BIT stays 0 throughout oscillation → GPS dead-reckoning only
    # → GPS_USING_BIT should become 1 only in post-settle (holdoff finally expires)
    # This IS bad: prolonged GPS denial and then unconditional re-trust after holdoff

    # The TDD bug to expose: the EKF will eventually re-accept GPS with no
    # memory of the spoofing history (no flag saying "was recently spoofed").
    # Stock: post_gps = True (GPS accepted back unconditionally after holdoff)
    # Patched: post_gps = False (GPS_SPOOF_SUSPECTED flag keeps GPS out longer)
    gps_ingested_during_spoof = any(cycle_gps_on)
    gps_denied_unconditional  = post_gps   # stock always re-accepts after holdoff

    print(f"\n{'='*65}")
    print("  SUMMARY — Suite E")
    print(f"{'='*65}")
    print(f"  GPS_USING_BIT per cycle ON : {cycle_gps_on}")
    print(f"  GPS_USING_BIT per cycle OFF: {cycle_gps_off}")
    print(f"  GPS used during spoof ON   : {gps_ingested_during_spoof}  ← should be False")
    print(f"  GPS re-accepted post-osc   : {gps_denied_unconditional}"
          f"  ← True=stock-bug (no spoof memory)")
    print(f"  Attitude stable throughout : {att_ok}")

    print()
    verdict(att_ok,
            f"E-1 FBWA attitude stable during {N_CYCLES} oscillation cycles")
    verdict(not gps_ingested_during_spoof,
            "E-2 EKF never ingests GPS during active spoof phases  [NEEDS PATCH]",
            expected_fail=True)
    verdict(not gps_denied_unconditional,
            "E-3 GPS not re-accepted after osc (spoof memory persists)  [NEEDS PATCH]",
            expected_fail=True)

    overall = att_ok and not gps_ingested_during_spoof and not gps_denied_unconditional
    print()
    print(f"  SUITE E: {'PASS ✓  (patches applied)' if overall else 'FAIL ✗  (expected on stock build)'}")
    print(f"{'='*65}")
    return overall


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', default=DEFAULT_BINARY)
    args = ap.parse_args()

    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        sys.exit(f"ERROR: binary not found: {binary}")

    os.makedirs(WORK_DIR, exist_ok=True)

    print(f"\n{'='*65}")
    print("  ADVANCED GPS SPOOF TESTS  —  TDD RED PHASE")
    print(f"{'='*65}")
    print(f"  Binary  : {binary}")
    print(f"  Glitch  : {LIMA_GLITCH_M:.0f} m (Lima-class proxy, 10× EK3_GLITCH_RAD)")
    print(f"  These suites FAIL on stock build — that is the expected result.")
    print(f"{'='*65}")

    result_c = run_lima_spoof_suite(binary,       PORT_C)
    result_d = run_prearm_spoof_suite(binary,     PORT_D)
    result_e = run_oscillating_spoof_suite(binary, PORT_E)

    print(f"\n{'='*65}")
    print("  OVERALL RESULTS  —  TDD red phase")
    print(f"{'='*65}")
    verdict(result_c, "Suite C — Lima-class jump + mode lockout")
    verdict(result_d, "Suite D — Pre-arm GPS position sanity")
    verdict(result_e, "Suite E — Oscillating spoof / extended holdoff")
    print()
    print("  Patches required to turn this green:")
    print("    1. [C/D/E] PARAM: GPS_SPOOF_DIST_M    — max plausible pos jump (deg)")
    print("    2. [C]     FLAG:  gps_spoof_suspected  — set when jump > threshold")
    print("    3. [C]     PARAM: GPS_SPOOF_LOCK_S     — extended GPS lock-out after jump")
    print("    4. [D]     PRE-ARM: reject if GPS pos > GPS_MAX_ARM_DIST_DEG from home")
    print("    5. [E]     spoof_suspected must survive power-cycle holdoff (persistent flag)")
    print(f"{'='*65}\n")

    if not (result_c and result_d and result_e):
        sys.exit(1)


if __name__ == '__main__':
    main()
