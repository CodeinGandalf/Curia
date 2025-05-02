# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy


def correctOffset(offset):
    """
    If there is an offset between the position of the can according to the
    LIDAR sensor and the camera picture, calculate the difference and correct
    the difference tracked by those two sensors.
    """
    rospy.loginfo("Hello from correctOffset")
    ...
