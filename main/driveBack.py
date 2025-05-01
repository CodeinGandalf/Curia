# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import gripper
import setBack


def driveBack(home, can):
    """
    Where am I on the map and where is my home spot?
    Drive back, based upon the difference of the position.
    """
    ...

    # If there is a can, lower the gripper and open it to release the can:
    if can is True:
        gripper.servoLower()
        gripper.gripperOpen()
        setBack.setBack()
