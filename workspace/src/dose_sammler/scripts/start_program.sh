#!/bin/bash

# 1. Starte den pigpio Daemon (falls nicht bereits aktiv)
if ! pgrep pigpiod > /dev/null
then
    echo "Starte pigpiod..."
    sudo pigpiod
else
    echo "pigpiod laeuft bereits."
fi

# 2. Navigiere in den Workspace-Ordner
cd ~/workspace || {
    echo "Workspace-Verzeichnis nicht gefunden!"
    exit 1
}

# 3. Source den Workspace
source devel/setup.bash

# 4. Starte das gewuenschte Launch-File
roslaunch dose_sammler dose_sammler.launch
