# GR-008 Gazebo Classic Model

Approximate Gazebo Classic 11 model for a 1 m turbine fixed-wing aircraft
(Swiwin SW80, rear-mounted, ~80 N max thrust) with ArduPilot SITL.

## Airframe summary

| Parameter | Value |
|-----------|-------|
| Wingspan | 1.0 m |
| Length | 1.0 m |
| AUW | ~3.5 kg |
| CG | 0.38 m from nose |
| Turbine | Swiwin SW80, rear-mounted |
| Cruise RPM | 140 000 (~64 N) |
| Max RPM | 156 000 (~80 N) |
| FC | Pixhawk 6X |
| Catapult angle | 12.3° nose-up |

## Servo assignments

| SERVO | Channel | Function | Joint |
|-------|---------|----------|-------|
| SERVO1 | 0 | Aileron left (function 4) | `left_aileron_joint` |
| SERVO2 | 1 | Aileron right (function 4, reversed) | `right_aileron_joint` |
| SERVO3 | 2 | Throttle (function 70) | `thrust_joint` (EFFORT, 0–80 N) |
| SERVO4 | 3 | Elevator (function 19) | `elevator_joint` |
| SERVO5 | 4 | Rudder (function 21) | `rudder_joint` |

## Launch (Linux / WSL2)

```bash
# Register the model path (run once per session, from repo root)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/Tools/simulation/gazebo/models

# Terminal 1 — start Gazebo with catapult world
gazebo --verbose Tools/simulation/gazebo/worlds/gr008_catapult.world

# Terminal 2 — start ArduPlane SITL connected to Gazebo
./build/sitl/bin/arduplane \
    --model gazebo-plane \
    --home 51.0,0.0,0,352 \
    --serial0 tcp:5760 \
    --defaults Tools/autotest/default_params/gazebo-gr008.parm

# Terminal 3 — GCS (MAVProxy / Mission Planner / QGC)
mavproxy.py --master tcp:127.0.0.1:5760
```

ArduPilot ↔ Gazebo communication uses UDP **9002** (servos AP→Gz) and **9003** (FDM Gz→AP).

## Notes on physics fidelity

`libLiftDragPlugin.so` provides per-surface linear lift/drag without
cross-coupling or dynamic derivatives (Cm_q, Cn_r, etc.).
ArduPilot's built-in `--model plane` is more complete for flight dynamics.
Use Gazebo when you need ground-contact physics or visual output;
use plain SITL for state-machine and PID testing.

All aerodynamic coefficients are first-order approximations.
Run AUTOTUNE / TECS tune before quantitative flight analysis.
