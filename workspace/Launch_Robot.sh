#!/bin/bash

# In das ROS-Arbeitsverzeichnis wechseln
cd ~/Curia/workspace || exit

# pigpiod mit Root-Rechten starten
sudo pigpiod

# ROS-Umgebung laden
source ~/Curia/workspace/devel/setup.bash

# ROS-Launch starten
roslaunch dose_samler dose_sammler.launch
