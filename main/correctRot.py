# -*- coding: utf-8 -*-
"""
Created on Thu May  1 12:20:59 2025

@author: Fuuli_FH_Pfloeck_wo_programiered!; Sagitarius A*
"""
import numpy as np
import matplotlib.pyplot as plt
import time

import inversKinematics as iK
import updatePWM as PWM
import driveToCan as dtC


def correctRot(posCan):
    """
    Meassure the rotation around the z axis from the current pose of the robot
    to the pose of the can and correct the rotation of it.
    """
    print("Hello")

    # Reglerparameter
    Kp = 0.7

    # Zielwert
    target_angle = 0.0

    # Initialwerte
    offsetAngle = posCan[0][1]

    error = target_angle - offsetAngle
    error_threshold = np.deg2rad(1)

    # Daten für Plot
    time_values = []
    angular_velocity_values = []
    angle_values = []

    start_time = time.time()
    last_time = start_time

    while abs(error) > error_threshold:
        now = time.time()
        dt = now - last_time
        last_time = now
        time_elapsed = now

        # Berechne Winkelfehler
        error = target_angle - offsetAngle

        # P-Regler: Winkelgeschwindigkeit
        angular_velocity = Kp * error

        # Begrenzung der Winkelgeschwindigkeit (z.B. ±1.0 rad/s)
        max_angular_velocity = 2.0
        angular_velocity = np.clip(
            angular_velocity, -max_angular_velocity, max_angular_velocity)

        # Simuliere Bewegung: Integration
        offsetAngle += angular_velocity * dt

        # Daten speichern
        time_values.append(time_elapsed)
        angular_velocity_values.append(angular_velocity)
        angle_values.append(offsetAngle)

        # Calculate the target wheel speed
        wheel_speeds = iK.mecanum_inv_kinematics(0, 0, angular_velocity)

        PWM.updatePWM(wheel_speeds)

        # Warte bis zur nächsten Schleife
        time.sleep(0.1)

    # Plot-Ergebnisse
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.plot(time_values, angular_velocity_values,
             label='Winkelgeschwindigkeit ω(t)')
    plt.xlabel('Zeit [s]')
    plt.ylabel('Winkelgeschwindigkeit [rad/s]')
    plt.title('P-Regler: Winkelgeschwindigkeit über Zeit')
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(time_values, angle_values, label='Winkel θ(t)', color='orange')
    plt.xlabel('Zeit [s]')
    plt.ylabel('Winkel [rad]')
    plt.title('P-Regler: Winkel über Zeit')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

    dx = 0
    dy = posCan[0][0]

    # Drive the robot to the can (linear movement):
    dtC.driveToCan(dx, dy)
