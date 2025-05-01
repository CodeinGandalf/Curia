# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import stopRobot as sR


def driveDefPath(home):
    # Control value for the leuze sensors:
    control = 0
    ...

    # As soon as both sensors see the can stop the robot and collect the can:
    if control == 2:
        sR.stopRobot(home)
    ...
