# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
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

    # Parameter for the controller:
    Kp = 0.7

    # Target angle based on the position of the robot:
    target_angle = 0.0

    # Angle from the can to the robot:
    offsetAngle = posCan[0][1]

    # Calculate the error of the angle:
    error = target_angle - offsetAngle

    # Use an error threshold of 1 deg:
    error_threshold = np.deg2rad(1)

    # Setup the variables for the plot:
    time_values = []
    angular_velocity_values = []
    angle_values = []

    # Get the current time:
    start_time = time.time()
    last_time = start_time

    # Control the error as long as it's greater then the error threshold:
    while abs(error) > error_threshold:
        # Get the current time while running:
        now = time.time()

        # Calculate the time that has passed:
        dt = now - last_time
        last_time = now
        time_elapsed = now

        # Calculate the current angle error:
        error = target_angle - offsetAngle

        # Approximate the angular velocity based on the error:
        angular_velocity = Kp * error

        # Calculate the target angular velocity:
        max_angular_velocity = 2.0
        angular_velocity = np.clip(
            angular_velocity, -max_angular_velocity, max_angular_velocity)

        # Calculate the new angle error
        offsetAngle += angular_velocity * dt

        # Safe the data for the plot:
        time_values.append(time_elapsed)
        angular_velocity_values.append(angular_velocity)
        angle_values.append(offsetAngle)

        # Calculate the targets for the wheel speeds:
        wheel_speeds = iK.mecanum_inv_kinematics(0, 0, angular_velocity)

        # Update the PWM for the engines:
        PWM.updatePWM(wheel_speeds)

        # Wait a short time befor the run through the loop:
        time.sleep(0.1)

    # Plot:
    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(time_values, angular_velocity_values,
             label='Angular velocity ω(t)')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular velocity  [rad/s]')
    plt.title('P-Regler: Angular velocity over the time')
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(time_values, angle_values, label='Angle θ(t)', color='orange')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [rad]')
    plt.title('P-Regler: Angle over the time')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

    # Set the target for dx to 0 and dy to drive towards the can:
    dx = 0
    dy = posCan[0][0]

    # Drive the robot to the can (linear movement on y axis):
    dtC.driveToCan(dx, dy)
