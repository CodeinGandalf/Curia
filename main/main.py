# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Fuuli_FH_Pfloeck_wo_programiered!; Sagitarius A*
"""
import numpy as np
import time
import matplotlib.pyplot as plt

import LIDAR_fake_data as LIDAR_fd
import searchCan as SC
import inversKinematics as iK
import checkIfCan as CIC
import updatePWM as PWM
import setBack
import servoLift
import servoGripper
import correctRot as cR
plt.close('all')


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


def startMapping(lidarData):
    """
    Take the LIDAR data and caluclate the mapping onto it.
    """
    ...


def driveToCan(dx, dy):
    """
    Drive the robot onto an linear path to the targeted position. Control
    the position in this part of the code with the data from the LIDAR to
    handle the drift from the engines.
    """
    ...


def correctOffset(offset):
    """
    If there is an offset between the position of the can according to the
    LIDAR sensor and the camera picture, calculate the difference and correct
    the difference tracked by those two sensors.
    """
    ...


def driveDefPath(home):
    # Check if both distance sensors say that they can see the can:
    control = 0
    ...

    # As soon as both sensors see the can stop the robot and collect the can:
    if control == 2:
        stopRobot(home)


def stopRobot(home):
    """
    Stop the robot, cause it has reached the position to grad the can.
    For that set the PWM targets for the engines to 0.
    """
    ...

    # Grab the can:
    grabCan()

    # Lift the can / gripper:
    servoUpper()

    # Bring the can to the home spot:
    driveBack(home, can=True)


def grabCan():
    """
    Close the gripper until it reaches the given target.
    """
    ...


def servoUpper():
    """
    Update the target of the servo to lift the gripper.
    """
    ...


def servoLower():
    ...


def openGripper():
    ...


def driveBack(home, can):
    """
    Where am I on the map and where is my home spot?
    Drive back, based upon the difference of the position.
    """
    ...
    # If there is a can, lower the gripper and open it to release the can:
    if can is True:
        servoLower()
        openGripper()
        setBack.setBack()


# Start the file if it's called as main file:
if __name__ == "__main__":
    # Get the fake LIDAR data:
    lidarData = LIDAR_fd.fake_data()

    """
    # Start to collect the LIDAR data:
    lidarData = collectLidarData()
    """

    # Calculate the map based upon the LIDAR data:
    home = startMapping(lidarData)

    # Set the starting variables to default:
    posCan = False
    isCan = False
    offsetCam = 100
    offsetLidar = 100
    camera_index = 0

    # Search for the pose of the potentional cans:
    posCan = SC.searchCan(lidarData)
    print(posCan)

    # Check if there are any cans in the LIDAR data:
    if posCan is not False:
        # Drive towards the can:
        cR.correctRot(posCan)

        # Check if there is a can based upon the LIDAR and cam data:
        isCan, offset = CIC.checkIfCan(posCan, camera_index,
                                       offsetCam, offsetLidar)
        print(f'isCan: {isCan}, offset: {offset}')

    # If there is a can; collect it:
    if isCan:
        # First check if there is an offset / correct it:
        if offset != 0:
            correctOffset(offset)

        # Move the robot into the position to grab the can:
        driveDefPath(home)
    else:
        # If there is no can, drive back to the home position:
        driveBack(home, can=False)
