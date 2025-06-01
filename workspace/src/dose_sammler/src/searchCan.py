#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import numpy as np
import rospy


# Define the function to search the can in the LIDAR data:
def searchCan(lidarData, can_diameter=67, tolerance=20):
    """
    Search for potential cans in LIDAR data by detecting sharp distance changes
    and evaluating the arc length between edges.

    Parameters:
    - lidarData: 2D numpy array with shape (2, N) where [0, :] = distances,
        [1, :] = angles

    Returns:
    - posCan: list of detected can pose as (distance, angle) tuples or False
    """
    rospy.loginfo("Hello from searchCan")
    posCan = []
    edges = []
    angle = []
    distances = lidarData[:, 0]
    angles = lidarData[:, 1]

    # Detect sharp changes in distance:
    for i in range(len(distances) - 1):
        diff = abs(distances[i+1] - distances[i])

        if diff > 20:
            edges.append(i)

    # Look for edge pairs:
    rospy.loginfo(f'{edges}')
    for i in range(0, len(edges) - 1, 2):
        start_idx = edges[i]
        end_idx = edges[i + 1]

        start_angle = angles[start_idx]
        end_angle = angles[end_idx]
        delta_angle_rad = end_angle - start_angle
        avg_radius = (distances[start_idx] + distances[end_idx]) / 2
        arc_length = avg_radius * delta_angle_rad
        expected_arc = np.pi * (can_diameter / 2.0)
        rospy.loginfo(f'pos arc len: {arc_length}, {expected_arc}')

        if abs(arc_length - expected_arc) < tolerance:
            center_idx = int((start_idx + end_idx) / 2)
            center_angle = np.angle(np.exp(1j * start_angle) +
                                    np.exp(1j * end_angle))
            center_distance = distances[center_idx]
            posCan.append((center_distance, center_angle))

    return posCan if posCan else False
