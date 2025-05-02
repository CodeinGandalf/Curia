# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import numpy as np
import matplotlib.pyplot as plt
import rospy


def fake_data():
    rospy.loginfo("Hello from LIDAR_fake_data")
    # Radius des Kreises in mm
    radius = 500

    # Bierdose Parameter
    distCan = 200  # Abstand der Bierdose vom Ursprung in mm
    d = 67  # Durchmesser der Bierdose in mm
    beer_can_radius = d / 2  # Radius der Bierdose in mm

    # Berechne den Winkelbereich, den die Bierdose blockiert (Bierdose als Circle)
    # Blockierter Winkel in Radianten
    alpha = 2 * np.arcsin(beer_can_radius / distCan)

    # Erstelle den Winkelbereich für den gesamten Kreis (0 bis 360 Grad)
    angles_deg = np.arange(0, 360)
    angles_rad = np.deg2rad(angles_deg)  # Umrechnung der Winkel in Radianten

    # Berechnung der x- und y-Koordinaten des Kreises
    x = radius * np.cos(angles_rad)
    y = radius * np.sin(angles_rad)

    # LIDAR-Daten modifizieren, um den Blockierungsbereich der Bierdose zu berücksichtigen
    for i in range(len(angles_rad)):
        # Prüfe, ob der Winkel im Bereich der Blockierung liegt
        if angles_rad[i] >= (np.pi - alpha / 2) and angles_rad[i] <= (np.pi + alpha / 2):
            # Setze den Punkt auf die Bierdose
            x[i] = distCan * np.cos(angles_rad[i])
            # Setze den Punkt auf die Bierdose
            y[i] = distCan * np.sin(angles_rad[i])

    lidarData = np.array([np.sqrt(x**2+y**2), angles_rad])

    # Plot
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, label="LIDAR Punkte")
    plt.scatter(0, 0, color='red', label="LIDAR Sensor (0,0)")
    plt.axis("equal")
    plt.xlabel("X [mm]")
    plt.ylabel("Y [mm]")
    plt.grid(True)
    plt.legend()
    plt.title("Simulierte LIDAR-Daten mit Bierdose im Sichtfeld")
    plt.show()

    return lidarData
