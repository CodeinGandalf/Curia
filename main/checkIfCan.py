# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 23:29:56 2025

@author: Fuuli_FH_Pfloeck_wo_programiered!; Sagitarius A*
"""

import searchCanCam
import numpy as np


def checkIfCan(posCan, camera_index, offsetCam, offsetLidar):
    """
    Check if both sensors see the can. If not let the bool for isCan at False
    and drive back. Otherwise start the algorithem to collect it.
    """

    center_x, center_y, canDetected = searchCanCam.find_best_can(
        posCan, camera_index, offsetCam, offsetLidar)

    print(canDetected)
    print(center_x)

    # Start with an offset of 0:
    if center_x is not False and canDetected is not False:
        cx = 640/2
        Z = posCan[0][0]*np.sin(posCan[0][1])
        fx = 4

        X_center = (center_x - cx)*Z/fx + offsetCam

        offset = X_center - (posCan[0][0]*np.cos(posCan[0][1]) - offsetLidar)
    else:
        offset = False

    # Return the state if there is a can and it's offset:
    return canDetected, offset
