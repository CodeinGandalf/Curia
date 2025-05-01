# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import gripper
import driveBack as dB


def stopRobot(home):
    """
    Stop the robot, cause it has reached the position to grad the can.
    For that set the PWM targets for the engines to 0.
    """
    # Grab the can:
    gripper.gripperClose()

    # Lift the can / gripper:
    gripper.servoUpper()

    # Bring the can to the home spot:
    dB.driveBack(home, can=True)

    ...
