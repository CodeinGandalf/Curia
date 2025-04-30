# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 17:21:25 2025

@author: Fuuli_FH_Pfloeck_wo_programiered!; Sagitarius A*
"""
import numpy as np


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
    posCan = []
    edges = []
    angle = []
    distances = lidarData[0, :]
    """
    Winkel des LIDARs muessen in rad sein!!!
    Die Auswertung aus dem Array der LIDAR Daten muessen mit den echten Daten
    von dem LIDAR Sensor abgeglichen werden!
    """
    angles = lidarData[1, :]

    # Detect sharp changes in distance (potential edges)
    for i in range(len(distances) - 1):
        diff = abs(distances[i+1] - distances[i])

        if diff > 20:
            edges.append(i)

    # Look for edge pairs (start-end of a potential object)
    for i in range(0, len(edges) - 1, 2):
        start_idx = edges[i]
        end_idx = edges[i + 1]

        start_angle = angles[start_idx]
        end_angle = angles[end_idx]
        delta_angle_rad = end_angle - start_angle
        avg_radius = (distances[start_idx] + distances[end_idx]) / 2
        arc_length = avg_radius * delta_angle_rad
        expected_arc = np.pi * (can_diameter / 2.0)

        deltaAngle = end_angle - start_angle

        if abs(arc_length - expected_arc) < tolerance:
            center_idx = int((start_idx + end_idx) / 2)
            center_angle = np.angle(
                np.exp(1j * start_angle) + np.exp(1j * end_angle))
            center_distance = distances[center_idx]
            posCan.append((center_distance, center_angle))

    return posCan if posCan else False
