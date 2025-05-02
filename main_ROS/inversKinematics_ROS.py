# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import numpy as np
import rospy


def mecanum_inv_kinematics(vx, vy, omega, wheelRadius=0.03, wheelBase=0.2, trackWidth=0.15):
    rospy.loginfo("Hello from inverse kinematics")
    L = wheelBase / 2
    W = trackWidth / 2

    # Matrix for the inverse kinematics
    J_inv = 1/wheelRadius*np.matrix([
        [1, -1, -(L + W)],
        [1, 1, (L + W)],
        [1, 1, -(L + W)],
        [1, -1, (L + W)]
    ])

    velocityVector = np.matrix([vx, vy, omega]).T
    wheelSpeeds = J_inv * velocityVector

    return wheelSpeeds.reshape(-1, 1)  # Convert matrix to a vertical array
