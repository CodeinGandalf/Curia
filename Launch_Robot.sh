#!/bin/bash

# In das ROS-Arbeitsverzeichnis wechseln
cd workspace || exit

# pigpiod mit Root-Rechten starten
if ! pgrep -x pigpiod > /dev/null; then
    sudo pigpiod
fi

# ROS-Umgebung laden
source devel/setup.bash

# ROS-Launch starten
roslaunch dose_sammler dose_sammler.launch
