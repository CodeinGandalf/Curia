#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy
import numpy as np

import searchCanCam


# Setup the function to check if there is a cam or not:
def checkIfCan(posCan, camera_index, offsetCam, offsetLidar):
    # Use the LIDAR data and search for the can in the picture:
    center_x, center_y, canDetected = searchCanCam.find_best_can(posCan, camera_index, offsetCam, offsetLidar)

    rospy.loginfo(f'Can detected? => {canDetected}')
    rospy.loginfo(f'Center x {center_x}')

    # Calculate the offset between the LIDAR data and the cam:
    if center_x is not False and canDetected is not False:
        # Define the variables for the camera model:
        cx = 640/2
        Z = posCan[0][0]*np.sin(posCan[0][1])
        fx = 4

        # Calculate the center from the camera picture:
        X_center = (center_x - cx)*Z/fx + offsetCam

        # Calculate the offset from the locations in the data sets:
        offset = X_center - (posCan[0][0]*np.cos(posCan[0][1]) - offsetLidar)
    else:
        offset = False

    # Return the state if there is a can and it's offset:
    return canDetected, offset
