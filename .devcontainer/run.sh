#!/bin/bash

add_source_bashrc () {
    if ! grep -q "source $1" ~/.bashrc; then
        echo "source $1" >> ~/.bashrc
    fi
}

set -e

source /opt/pal/gallium/setup.bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt update

# get network functionalities such as ping and ip
sudo apt install -y iproute2
sudo apt install -y iputils-ping

# --- AUDIO CONFIGURATION START ---

echo "--- Setting up audio for the container ---"

# Install PulseAudio client utilities and ALSA utilities
sudo apt-get update
sudo apt-get install -y pulseaudio-utils alsa-utils

#audio

sudo apt install -y python3-pyaudio
# Add the 'root' user (your containerUser) to the 'audio' group.
echo "Adding root user to the 'audio' group..."
sudo usermod -aG audio root

# Set appropriate permissions for the PulseAudio socket within the container.
echo "Setting permissions for PulseAudio socket..."
chmod 777 "/run/user/$(id -u)/pulse" || true

# IMPORTANT: Fix ownership of the mounted PulseAudio cookie
# This command changes the owner of the mounted cookie file to 'root'
# The `ls -l` output showed it was owned by 'user user', preventing root from reading it.
if [ -f "/root/.config/pulse/cookie" ]; then
    echo "Fixing ownership of PulseAudio cookie..."
    chown root:root /root/.config/pulse/cookie
    chmod 600 /root/.config/pulse/cookie # Ensure only root can read/write it
else
    echo "PulseAudio cookie not found at /root/.config/pulse/cookie. Skipping ownership fix."
fi

echo "--- Audio setup complete ---"

# --- AUDIO CONFIGURATION END ---


sudo rosdep init
sudo rosdep update


add_source_bashrc "/opt/pal/gallium/setup.bash"