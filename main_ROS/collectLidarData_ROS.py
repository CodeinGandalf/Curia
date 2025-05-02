# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy


def collectLidarData():
    """
    Set the motor targets of the robot to an constant rotation speed.
    As soon as the rot speed has been reached start the mapping with the LIDAR.
    For that the time delay of between two meassurements have to be tracked,
    so the passed angle by the robot can be calculated by the angle speed*dt.
    So every point from the LIDAR scanner has an angle. This angel musst be
    combined with the passed angel of the robot, normalized to 2 Pi.
    """
    ...
    lidarData = None
    rospy.loginfo("Hello from collectLidarData")
    return lidarData
