# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import numpy as np
import time
import matplotlib.pyplot as plt
import updatePWM as PWM
import inversKinematics as iK


def setBack():
    start_time = time.time()
    v_max = -1
    target_velocity = 0

    velocity_values = []
    time_values = []

    while True:
        elapsed = time.time() - start_time

        if 0 <= elapsed <= 5:
            target_velocity = v_max * (elapsed / 5)
        elif 5 < elapsed <= 10:
            target_velocity = v_max * ((10 - elapsed) / 5)
        else:
            target_velocity = 0.0
            PWM.updatePWM([0.0, 0.0, 0.0, 0.0])
            break

        velocity_values.append(target_velocity)
        time_values.append(elapsed)

        dx = 0.0
        dy = target_velocity
        rot = 0.0

        # Calculate the target wheel speed
        wheel_speeds = iK.mecanum_inv_kinematics(dx, dy, rot)

        PWM.updatePWM(wheel_speeds)

        time.sleep(0.01)

    # Plot erstellen
    plt.figure(figsize=(12, 6))

    # Geschwindigkeit
    plt.plot(time_values, velocity_values,
             label="Geschwindigkeit (m/s)", color="blue")
    plt.scatter(time_values[::50], np.array(
        velocity_values)[::50], color="blue", s=15)

    plt.axhline(0, color='black', linestyle='--', linewidth=0.5)
    plt.title("Rückwärtsbewegung: Geschwindigkeit, Position und Beschleunigung")
    plt.xlabel("Zeit (s)")
    plt.ylabel("Wert")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
