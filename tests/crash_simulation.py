#!/usr/bin/env python3
"""
GR-005 catapult crash simulation.

Reproduces the control and physical dynamics from GR-005_2_Crush.BIN.
Shows the integrator windup mechanism, the catapult launch, AHRS corruption,
and the resulting nose-down crash — with and without Fix #1.

╔══════════════════════════════════════════════════════════════════════════════╗
║  INPUTS (what the drone receives from the outside world)                    ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  IMU.AccX    body-frame forward acceleration (m/s²)                         ║
║              idle on rail: ~2 m/s²  │  catapult peak: 156 m/s² (≈16G)      ║
║  IMU.GyrY    pitch rate (rad/s)                                             ║
║              idle on rail: ~0  │  catapult: +2.0 nose-up → -2.4 nose-down  ║
║  ARSP        pitot equivalent airspeed (m/s)                                ║
║              rail: 4–11 m/s — turbine exhaust blows across pitot tube       ║
║  BARO.Alt    barometric altitude (m)                                        ║
║              rail: 0–4 m — turbine exhaust raises static port pressure      ║
║  RCIN.C2     elevator stick PWM  (1500 = neutral, held neutral throughout)  ║
║  RCIN.C3     throttle stick PWM  (1285 idle → 1899 max @ t≈148s)           ║
║  GPS.Status  1 = alive, no fix  (country-wide GPS jamming)                  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  OUTPUTS (what the FC sends to actuators)                                   ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  RCOU.C2 / AETR.Elev   elevator servo (−45° to +45°)                       ║
║                         pre-launch: −23° → −45° (saturated)                ║
║  RCOU.C3 / AETR.Thr    turbine throttle %  (13% idle → 99% full)           ║
║  EFI.Rpm  via CAN       turbine RPM setpoint  (50 k idle → 144 k full)     ║
║  AETR.Ail               aileron ≈ 0°  (wings level on rail)                 ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  HOW THE CRASH HAPPENS                                                      ║
║                                                                             ║
║  1. Plane on rail at +12° pitch; turbine spins from idle to 144 k RPM.     ║
║  2. Turbine exhaust blows across pitot tube → airspeed reading rises 4→11   ║
║     m/s even though the plane is stationary.                                ║
║  3. At airspeed > 8.5 m/s (= 0.5 × AIRSPEED_MIN = 0.5 × 17 m/s),         ║
║     the pitch controller's underspeed_lock clears.                          ║
║  4. Plane cannot rotate on the rail → persistent pitch-rate error →        ║
║     integrator I winds from 0° to −38.1° (IMAX) in ≈10 s.                 ║
║  5. Catapult fires: AccX peaks at 156 m/s² (≈16G) for 0.2 s.              ║
║  6. 19G impulse corrupts AHRS: it reports +23.8° false nose-up pitch.      ║
║  7. Elevator = FF(−17.6°) + P(−11.6°) + I(−38.1°) = −67° → clamped −45°. ║
║  8. Plane pitches from +12° to −43° in 0.7 s → crash.                      ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import math


# ─── Parameters from GR-005_2_Crush.BIN PARM log ────────────────────────────

PTCH_RATE_P    = 0.040
PTCH_RATE_I    = 0.150
PTCH_RATE_FF   = 0.345
PTCH_RATE_IMAX = 0.666          # rad  (= 38.16°)

SCALING_SPEED  = 15.0           # m/s
AIRSPEED_MIN   = 17.0           # m/s  — AIRSPEED_MIN param
UNDERSPEED_THR = 0.5 * AIRSPEED_MIN   # 8.5 m/s — pitch controller threshold

ELEV_LIMIT     = 45.0           # ° (±4500 centidegrees in ArduPlane)
TKOFF_THR_MINACC = 30.0         # m/s²  — catapult detection threshold

# Aircraft physical constants (GR-005 estimates)
MASS           = 8.5            # kg
WING_AREA      = 0.35           # m²
CHORD          = 0.25           # m  — mean aerodynamic chord
I_PITCH        = 1.4            # kg·m²  — pitch moment of inertia
AIR_DENSITY    = 1.225          # kg/m³ (sea level)

# Aerodynamic coefficients fitted to reproduce observed log dynamics:
#   - real crash: pitch went from +12° to −43° in ≈0.7 s post-launch
#   - real pitch rate at peak: −2.4 rad/s (from IMU.GyrY)
# Sign convention: positive elevator (trailing-edge up) = positive (nose-up) moment.
CM_ALPHA       = -0.45          # pitch stiffness (per rad AoA) — statically stable
CM_ELEV        =  0.80          # pitch moment per rad elevator deflection
CM_Q           = -4.0           # pitch damping (per dimensionless q̂ = q·c/(2v))

# ─── Catapult pulse — extracted from GR-005_2_Crush.BIN IMU log ─────────────
# Measured AccX at 240 Hz around t = 197.72 – 197.92 s
CATAPULT_PULSE = [          # (segment_duration_s, AccX_m_s2)
    (0.040,  44.97),        # first spike
    (0.040, 156.53),        # ≈ 16G peak
    (0.040, 129.35),
    (0.040, 117.92),
    (0.040,  12.46),        # back to near-normal
]

# ─── AHRS corruption model during 19G impulse ────────────────────────────────
# AHRS DCM confuses forward acceleration with gravity, reporting false nose-up.
# Observed from ATT log (interpolated):
#   t + 0.00 s : ATT.Pitch = +12.0°  (true, pre-launch)
#   t + 0.12 s : ATT.Pitch = +23.8°  (AHRS false nose-up peak)
#   t + 0.40 s : ATT.Pitch =  +2.5°  (AHRS recovering)
#   t + 0.68 s : ATT.Pitch = −33.7°  (crash; AHRS now reflects real pitch)
_AHRS_CORRUPTION = [
    (0.00,  0.0),           # (t_since_launch, extra_pitch_error_deg)
    (0.12, +11.8),          # peak AHRS over-read (23.8 − 12.0)
    (0.40,  -9.5),          # AHRS under-reads as it recovers
    (0.68,   0.0),          # fully recovered
]

def ahrs_bias_deg(t_since_launch):
    """Returns the extra pitch (deg) AHRS adds due to 19G corruption."""
    pts = _AHRS_CORRUPTION
    if t_since_launch <= pts[0][0]:
        return pts[0][1]
    if t_since_launch >= pts[-1][0]:
        return pts[-1][1]
    for i in range(len(pts) - 1):
        t0, b0 = pts[i]
        t1, b1 = pts[i + 1]
        if t0 <= t_since_launch <= t1:
            frac = (t_since_launch - t0) / (t1 - t0)
            return b0 + frac * (b1 - b0)
    return 0.0


# ─── Pitch rate controller ────────────────────────────────────────────────────
# Mirrors AP_FW_Controller::_get_rate_out + AP_PitchController::is_underspeed.

class PitchRateController:
    def __init__(self):
        self._i = 0.0
        self._last_out = 0.0

    def reset_I(self):
        self._i = 0.0

    @property
    def integrator(self):
        return self._i

    def update(self, target_rate_dps, actual_rate_dps, airspeed_ms, dt):
        """Returns elevator demand (degrees, clamped to ±ELEV_LIMIT)."""
        # Speed scaler: clamped to [0.5, 2.0]
        scaler = min(2.0, max(0.5, SCALING_SPEED / max(airspeed_ms, 0.5)))

        # Underspeed protection: lock integrator below threshold
        underspeed = airspeed_ms <= UNDERSPEED_THR
        limit_I    = (abs(self._last_out) >= ELEV_LIMIT) or underspeed

        # Convert to radians and apply scaler² (ArduPlane convention)
        target_rad  = math.radians(target_rate_dps)
        actual_rad  = math.radians(actual_rate_dps)
        scaled_err  = (target_rad - actual_rad) * scaler * scaler

        ff_deg = math.degrees(PTCH_RATE_FF * target_rad / scaler)
        p_deg  = math.degrees(PTCH_RATE_P * scaled_err)

        if not limit_I:
            imax_deg = math.degrees(PTCH_RATE_IMAX)
            di = math.degrees(PTCH_RATE_I * scaled_err * dt)
            self._i = max(-imax_deg, min(imax_deg, self._i + di))

        out = ff_deg + p_deg + self._i
        self._last_out = out
        return max(-ELEV_LIMIT, min(ELEV_LIMIT, out))


def outer_pitch_demand(nav_pitch_deg, ahrs_pitch_deg):
    """Returns demanded pitch rate (°/s) — simplified PTCH2SRV outer loop."""
    TCONST = 0.5        # PTCH2SRV_TCONST
    MAX_RATE = 60.0     # °/s  (unlimited when PTCH2SRV_RMAX_DN/UP = 0)
    return max(-MAX_RATE, min(MAX_RATE, (nav_pitch_deg - ahrs_pitch_deg) / TCONST))


def pitch_angular_accel(pitch_deg, pitch_rate_dps, velocity_ms, elev_deg):
    """Returns pitch angular acceleration (°/s²) — simplified 2-D aerodynamics."""
    if velocity_ms < 1.0:
        return 0.0
    q_bar   = 0.5 * AIR_DENSITY * velocity_ms ** 2   # dynamic pressure (Pa)
    aoa_rad = math.radians(pitch_deg)                 # simplified AoA ≈ pitch
    elev_rad = math.radians(elev_deg)
    q_hat   = math.radians(pitch_rate_dps) * CHORD / (2.0 * velocity_ms)

    cm = CM_ALPHA * aoa_rad + CM_ELEV * elev_rad + CM_Q * q_hat
    moment = q_bar * WING_AREA * CHORD * cm           # N·m
    return math.degrees(moment / I_PITCH)             # °/s²


# ─── Main simulation loop ─────────────────────────────────────────────────────

def run(fix1_active=False, fix2_active=False):
    """
    Simulate from arm to 1.5 s post-launch.
    fix1_active: reset integrators on AccX > TKOFF_THR_MINACC (Fix #1)
    fix2_active: reset integrators when airspeed < 2 m/s AND alt < 5 m (Fix #2)
    Returns list of per-step state dicts.
    """
    DT = 0.02          # 50 Hz

    # State
    pitch_deg      = 12.0   # on catapult rail at +12° pitch
    pitch_rate_dps = 0.0
    velocity_ms    = 3.0    # slight headwind pre-launch
    on_rail        = True
    nav_pitch_deg  = 0.0    # FBWA: FC targets level flight

    ctrl = PitchRateController()
    history = []

    # Timing anchors (seconds since arm, matching the log)
    THROTTLE_PUSH_T = 148.0   # pilot pushes throttle to max
    LAUNCH_T        = 159.0   # AccX spike (log: 197.74 − 38.76 = 158.98 s)

    # Catapult pulse state
    launch_t        = None
    pulse_idx       = 0
    pulse_dt_acc    = 0.0

    # Fix #1 reset window
    reset_until     = None

    def airspeed_at(t):
        """Pitot reading: real airspeed + turbine exhaust contribution."""
        if t < THROTTLE_PUSH_T:
            return 3.5   # light wind, turbine at idle
        ramp = min(1.0, (t - THROTTLE_PUSH_T) / 12.0)
        return 3.5 + 7.5 * ramp   # 3.5 → 11.0 m/s over 12 s

    def baro_alt_at(t):
        if t < THROTTLE_PUSH_T:
            return 0.0
        return min(4.0, (t - THROTTLE_PUSH_T) / 3.0)   # 0 → 4 m from exhaust

    def baro_climb_at(t):
        if t < THROTTLE_PUSH_T or t > THROTTLE_PUSH_T + 12.0:
            return 0.0
        return 4.0 / 12.0   # ≈ 0.33 m/s

    t = 0.0
    while t < LAUNCH_T + 1.5:
        airspeed  = airspeed_at(t) if on_rail else max(velocity_ms, 5.0)
        baro_alt  = baro_alt_at(t)
        baro_clmb = baro_climb_at(t)

        # ── Fix #2: airspeed-gated pre-takeoff integrator reset ──────────────
        if fix2_active and on_rail:
            if airspeed < 2.0 and baro_alt < 5.0 and abs(baro_clmb) < 0.5:
                ctrl.reset_I()

        # ── Launch ───────────────────────────────────────────────────────────
        if t >= LAUNCH_T and launch_t is None:
            launch_t  = t
            on_rail   = False
            velocity_ms = 12.0   # headwind + turbine pre-launch contribution

        t_post_launch = (t - launch_t) if launch_t is not None else None

        # ── Catapult acceleration pulse ───────────────────────────────────────
        acc_x = 0.0
        if t_post_launch is not None and pulse_idx < len(CATAPULT_PULSE):
            seg_dt, seg_acc = CATAPULT_PULSE[pulse_idx]
            acc_x = seg_acc
            pulse_dt_acc += DT
            if pulse_dt_acc >= seg_dt:
                velocity_ms += seg_acc * seg_dt   # ΔV from segment
                pulse_dt_acc = 0.0
                pulse_idx += 1

        # ── Fix #1: reset on catapult detection ──────────────────────────────
        if fix1_active and acc_x > TKOFF_THR_MINACC:
            if reset_until is None:
                reset_until = t + 0.300
        if reset_until is not None:
            if t < reset_until:
                ctrl.reset_I()
            else:
                reset_until = None

        # ── AHRS pitch (corrupted during 19G impulse) ─────────────────────────
        if t_post_launch is not None:
            ahrs_pitch = pitch_deg + ahrs_bias_deg(t_post_launch)
        else:
            ahrs_pitch = pitch_deg

        # ── Outer pitch loop → demanded rate ─────────────────────────────────
        target_rate = outer_pitch_demand(nav_pitch_deg, ahrs_pitch)
        measured_rate = 0.0 if on_rail else pitch_rate_dps

        # ── Inner pitch rate PID ──────────────────────────────────────────────
        elev_deg = ctrl.update(target_rate, measured_rate, airspeed, DT)

        # ── Pitch dynamics (only after leaving rail) ──────────────────────────
        if not on_rail:
            # Brief nose-up impulse from catapult rail reaction (matches GyrY log)
            catapult_pitch_up = 0.0
            if t_post_launch is not None and t_post_launch < 0.12:
                frac = 1.0 - t_post_launch / 0.12
                catapult_pitch_up = frac * 110.0   # °/s nose-up (log: GyrY ≈ +2 rad/s)

            aero_accel = pitch_angular_accel(pitch_deg, pitch_rate_dps, velocity_ms, elev_deg)
            pitch_rate_dps += (aero_accel + catapult_pitch_up * (0 if t_post_launch > 0.08 else 1)) * DT
            pitch_deg      += pitch_rate_dps * DT
            velocity_ms     = max(velocity_ms, 10.0)

        # ── Record ───────────────────────────────────────────────────────────
        history.append({
            't':          t,
            'pitch':      pitch_deg,
            'ahrs_pitch': ahrs_pitch,
            'rate':       pitch_rate_dps,
            'elev':       elev_deg,
            'I':          ctrl.integrator,
            'P':          elev_deg - ctrl.integrator - math.degrees(PTCH_RATE_FF *
                           math.radians(outer_pitch_demand(nav_pitch_deg, ahrs_pitch)) /
                           min(2.0, max(0.5, SCALING_SPEED / max(airspeed, 0.5)))),
            'airspeed':   airspeed,
            'acc_x':      acc_x,
            'on_rail':    on_rail,
        })
        t += DT

    return history


# ─── Display helpers ──────────────────────────────────────────────────────────

def print_phase(label, history, t_start, t_end, step_s=1.0):
    samples = [h for h in history if t_start <= h['t'] <= t_end]
    last_t = -1e9
    print(f"\n{'─'*86}")
    print(f" {label}")
    print(f"{'─'*86}")
    print(f"  {'t':>7}  {'Airspd':>7}  {'Pitch':>8}  {'AHRS':>8}  "
          f"{'Rate°/s':>8}  {'Elev°':>7}  {'I°':>8}  {'AccX':>7}  Note")
    for h in samples:
        if h['t'] - last_t < step_s - 1e-6:
            continue
        last_t = h['t']

        note = ''
        if h['on_rail']:
            note = 'underspeed-locked' if h['airspeed'] <= 8.5 else 'I winding'
        else:
            if h['pitch'] < -25:
                note = '*** CRASH ***'
            elif abs(h['ahrs_pitch'] - h['pitch']) > 4:
                note = 'AHRS confused'

        print(f"  {h['t']:>7.2f}  {h['airspeed']:>7.2f}  {h['pitch']:>+8.2f}  "
              f"{h['ahrs_pitch']:>+8.2f}  {h['rate']:>+8.1f}  "
              f"{h['elev']:>+7.1f}  {h['I']:>+8.3f}  {h['acc_x']:>7.1f}  {note}")


def print_launch_comparison(h_orig, h_fix1, h_both, t0=158.8, t1=160.2):
    print(f"\n{'='*100}")
    print("  LAUNCH COMPARISON")
    print(f"{'='*100}")
    hdr = (f"  {'t(s)':>6}  │  {'──── ORIGINAL (no fix) ────':^30}  │  "
           f"{'──── FIX #1 only ────':^30}  │  {'FIX #1+#2':^12}")
    print(hdr)
    print(f"  {'':>6}  │  {'Pitch':>7} {'Elev':>7} {'I':>8} {'Elev-I':>8}  │  "
          f"{'Pitch':>7} {'Elev':>7} {'I':>8} {'Elev-I':>8}  │  {'Pitch':>7} {'Elev':>7}")
    print("  " + "─" * 97)

    def nn(lst, tv):
        return min(lst, key=lambda h: abs(h['t'] - tv))

    step = 0.10
    t = t0
    while t <= t1 + 1e-9:
        o = nn(h_orig, t)
        f = nn(h_fix1, t)
        b = nn(h_both, t)
        print(f"  {t:>6.2f}  │  {o['pitch']:>+7.2f} {o['elev']:>+7.1f} "
              f"{o['I']:>+8.3f} {o['elev']-o['I']:>+8.3f}  │  "
              f"{f['pitch']:>+7.2f} {f['elev']:>+7.1f} "
              f"{f['I']:>+8.3f} {f['elev']-f['I']:>+8.3f}  │  "
              f"{b['pitch']:>+7.2f} {b['elev']:>+7.1f}")
        t = round(t + step, 3)


def main():
    print("=" * 86)
    print("  GR-005 CATAPULT CRASH SIMULATION")
    print("  Derived from GR-005_2_Crush.BIN  (ArduPlane V4.6.2, Pixhawk 6X)")
    print("=" * 86)

    h_orig = run(fix1_active=False, fix2_active=False)
    h_fix1 = run(fix1_active=True,  fix2_active=False)
    h_both = run(fix1_active=True,  fix2_active=True)

    # ── Phase 1: standby — show integrator windup ─────────────────────────────
    print_phase(
        "PHASE 1 — STANDBY: integrator windup mechanism  (t = 143 s to 159 s)",
        h_orig, 143.0, 159.2, step_s=1.0)

    print(f"""
  WINDUP MECHANISM:
    AIRSPEED_MIN = {AIRSPEED_MIN:.0f} m/s → underspeed threshold = {UNDERSPEED_THR:.1f} m/s
    (AP_PitchController::is_underspeed: return get_airspeed() <= 0.5*AIRSPEED_MIN)

    While airspeed ≤ 8.5 m/s: integrator locked — even though pitch error is large.
    When turbine exhaust raises pitot reading above 8.5 m/s:
      lock clears → integrator accumulates at full rate (~3.5°/s).
    Time from lock-clear to IMAX saturation: ≈10 s.
    At IMAX: I = −{math.degrees(PTCH_RATE_IMAX):.1f}°  (PTCH_RATE_IMAX = {PTCH_RATE_IMAX} rad).
""")

    # ── Phase 2: launch ───────────────────────────────────────────────────────
    print_phase(
        "PHASE 2 — CATAPULT LAUNCH, ORIGINAL  (t = 158.8 to 160.2 s)",
        h_orig, 158.8, 160.2, step_s=0.1)

    print_phase(
        "PHASE 2 — CATAPULT LAUNCH, WITH FIX #1  (t = 158.8 to 160.2 s)",
        h_fix1, 158.8, 160.2, step_s=0.1)

    # ── Side-by-side comparison ───────────────────────────────────────────────
    print_launch_comparison(h_orig, h_fix1, h_both)

    # ── Final verdict ─────────────────────────────────────────────────────────
    def min_pitch_post(history):
        post = [h['pitch'] for h in history if not h['on_rail']]
        return min(post) if post else 0.0

    mp_o = min_pitch_post(h_orig)
    mp_f = min_pitch_post(h_fix1)
    mp_b = min_pitch_post(h_both)

    print(f"\n{'='*86}")
    print("  VERDICT")
    print(f"{'='*86}")
    print(f"  Original (no fix) : min pitch = {mp_o:+.1f}°  "
          + ("→ CRASH" if mp_o < -25 else "→ survived"))
    print(f"  Fix #1 only       : min pitch = {mp_f:+.1f}°  "
          + ("→ CRASH" if mp_f < -25 else "→ survived"))
    print(f"  Fix #1 + Fix #2   : min pitch = {mp_b:+.1f}°  "
          + ("→ CRASH" if mp_b < -25 else "→ survived"))

    print(f"""
  IMPORTANT CAVEAT ABOUT FIX #2:
    Fix #2 resets integrators when airspeed < 2 m/s.
    BUT turbine exhaust raises pitot reading to 4–11 m/s on the stationary rail.
    → Fix #2 threshold (2 m/s) is breached almost immediately after throttle push.
    → Windup starts ~8.5 s earlier than with Fix #2, reaching IMAX at same moment.
    Fix #2 alone does NOT prevent windup in the turbine-on-rail scenario.
    Fix #2 only helps when wind is calm AND turbine is at idle (low exhaust).

  FIX #1 ELEVATOR REDUCTION AT LAUNCH:
    Original:  FF + P + I = −11° + (−12°) + (−38°) = −61° → saturated at −45°
    Fix #1:    FF + P + I = −11° + (−12°) +    0°  = −23° → NOT saturated
    Margin gained: +22°  (almost halves the nose-down elevator command)
    This gives the aircraft 0.3 s to recover from AHRS confusion without
    riding the elevator stop.
""")
    print("=" * 86)


if __name__ == '__main__':
    main()
