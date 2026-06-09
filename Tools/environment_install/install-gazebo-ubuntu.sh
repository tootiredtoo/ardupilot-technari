#!/usr/bin/env bash
# Install Gazebo Harmonic and ardupilot_gazebo plugin on Ubuntu 22.04/24.04.
# Run once from any directory; re-running is safe (all steps are idempotent).
set -e

GZ_VERSION=harmonic
PLUGIN_DIR="$HOME/ardupilot_gazebo"

# ── 1. Gazebo Harmonic via OSRF apt repo ───────────────────────────────────
if ! command -v gz &>/dev/null; then
    sudo apt-get install -y wget lsb-release gnupg

    wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /tmp/pkgs-osrf-archive-keyring.gpg
    sudo cp /tmp/pkgs-osrf-archive-keyring.gpg \
        /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y gz-harmonic
else
    echo "gz already installed: $(gz sim --version 2>&1 | head -1)"
fi

# ── 2. ardupilot_gazebo plugin build dependencies ─────────────────────────
sudo apt-get install -y \
    libgz-sim8-dev \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl

# ── 3. Clone and build ardupilot_gazebo plugin ────────────────────────────
if [ ! -d "$PLUGIN_DIR" ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo "$PLUGIN_DIR"
fi

cmake -B "$PLUGIN_DIR/build" "$PLUGIN_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -C "$PLUGIN_DIR/build" -j"$(nproc)"

# ── 4. Persist environment variables ──────────────────────────────────────
BASHRC="$HOME/.bashrc"
MARKER="# ArduPilot + Gazebo"

if ! grep -q "$MARKER" "$BASHRC" 2>/dev/null; then
    cat >> "$BASHRC" << EOF

$MARKER
export GZ_SIM_SYSTEM_PLUGIN_PATH=\$HOME/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=\$HOME/ardupilot_gazebo/models:\$HOME/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH
EOF
    echo "Environment variables added to $BASHRC"
fi

echo ""
echo "Setup complete. Open a new shell (or 'source ~/.bashrc') then run:"
echo "  Tools/scripts/gazebo_sitl.sh ArduCopter iris_runway"
