#!/usr/bin/env bash
# Launch ArduPilot SITL paired with Gazebo Harmonic.
#
# Usage:
#   gazebo_sitl.sh <vehicle> <world> [extra sim_vehicle args]
#
#   vehicle  ArduCopter | ArduPlane | Rover | ArduSub
#   world    iris_runway | iris_warehouse | zephyr_runway | zephyr_parachute
#            (names under ~/ardupilot_gazebo/worlds, without .sdf extension)
#
# Examples:
#   gazebo_sitl.sh ArduCopter iris_runway
#   gazebo_sitl.sh ArduPlane  zephyr_runway --speedup 2
#
# The script starts Gazebo headless (no GUI) in the background, then launches
# SITL connected via the JSON protocol used by the ardupilot_gazebo plugin.
# Kill both with Ctrl-C.

set -e

VEHICLE="${1:?Usage: $0 <vehicle> <world> [sim_vehicle_args...]}"
WORLD="${2:?Usage: $0 <vehicle> <world> [sim_vehicle_args...]}"
shift 2

PLUGIN_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
WORLD_FILE="$PLUGIN_DIR/worlds/${WORLD}.sdf"

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: world file not found: $WORLD_FILE"
    echo "Available worlds:"
    ls "$PLUGIN_DIR/worlds/"
    exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$PLUGIN_DIR/models:$PLUGIN_DIR/worlds:$GZ_SIM_RESOURCE_PATH"

# Map vehicle → frame name used by sim_vehicle.py
case "$VEHICLE" in
    ArduCopter) FRAME="gazebo-iris" ;;
    ArduPlane)  FRAME="gazebo-zephyr" ;;
    Rover)      FRAME="gazebo-rover" ;;
    ArduSub)    FRAME="gazebo-bluerov2" ;;
    *)          FRAME="$VEHICLE" ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "Starting Gazebo headless: $WORLD_FILE"
gz sim -v4 -s -r "$WORLD_FILE" &
GZ_PID=$!

# Give Gazebo a moment to bind its socket before SITL connects
sleep 3

echo "Starting ArduPilot SITL: vehicle=$VEHICLE frame=$FRAME"
"$REPO_ROOT/build/sitl/bin/${VEHICLE,,}" \
    --model JSON \
    --speedup 1 \
    --sim-address=127.0.0.1 \
    -I0 \
    "$@" &
SITL_PID=$!

trap 'echo "Stopping..."; kill $GZ_PID $SITL_PID 2>/dev/null; wait' INT TERM

wait $SITL_PID
kill $GZ_PID 2>/dev/null
