# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Fuuli_FH_Pfloeck_wo_programiered!; Sagitarius A*
"""


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


def startMapping():
    """
    Take the LIDAR data and caluclate the mapping onto it.
    """
    ...


def searchCan():
    """
    Get the LIDAR data and search for potential cans. If there is no can
    let the variable for the pose be False.
    """
    # Define the pose of the can as false:
    posCan = False
    ...

    # Return the pose of the can:
    return posCan


def correctRot():
    """
    Meassure the rotation around the z axis from the current pose of the robot
    to the pose of the can and correct the rotation of it.
    """
    ...


def driveToCan():
    """
    Drive the robot onto an linear path to the targeted position. Control
    the position in this part of the code with the data from the LIDAR to
    handle the drift from the engines.
    """
    ...


def checkIfCan():
    """
    Check if both sensors see the can. If not let the bool for isCan at False
    and drive back. Otherwise start the algorithem to collect it.
    """
    # Set the isCan bool to false and update it if there is a can in the data:
    isCan = False

    # Start with an offset of 0:
    offset = 0
    ...

    # Return the state if there is a can and it's offset:
    return isCan, offset


def correctOffset():
    """
    If there is an offset between the position of the can according to the
    LIDAR sensor and the camera picture, calculate the difference and correct
    the difference tracked by those two sensors.
    """
    ...


def driveDefPath():
    # Check if both distance sensors say that they can see the can:
    control = 0
    ...

    # As soon as both sensors see the can stop the robot and collect the can:
    if control == 2:
        stopRobot()


def stopRobot():
    """
    Stop the robot, cause it has reached the position to grad the can.
    For that set the PWM targets for the engines to 0.
    """
    ...
    # Grab the can:
    grabCan()

    # Lift the can / gripper:
    liftCan()

    # Bring the can to the home spot:
    driveBack(can=True)


def grabCan():
    """
    Close the gripper until it reaches the given target.
    """
    ...


def liftCan():
    """
    Update the target of the servo to lift the gripper.
    """
    ...


def driveBack(can):
    """
    Where am I on the map and where is my home spot?
    Drive back, based upon the difference of the position.
    """
    ...
    # If there is a can, lower the gripper and open it to release the can:
    if can:
        lowerCan()
        openGripper()
        setBack()


def lowerCan():
    ...


def openGripper():
    ...


def setBack():
    ...


# Start the file if it's called as main file:
if __name__ == "__main__":
    # Start to collect the LIDAR data:
    collectLidarData()

    # Calculate the map based upon the LIDAR data:
    startMapping()

    # Search for the pose of the potentional cans:
    posCan = searchCan()

    # Check if there are any cans in the LIDAR data:
    if posCan is not False:
        # First correct the rotation around the z axis:
        correctRot()

        # Drive the robot to the can (linear movement):
        driveToCan()

        # Check if there is a can based upon the LIDAR and cam data:
        isCan, offset = checkIfCan()

    # If there is a can, then collect it:
    if isCan:
        # First check if there is an offset / correct it:
        if offset != 0:
            correctOffset()

        # Move the robot into the position to grab the can:
        driveDefPath()

    else:
        # If there is no can, drive back to the home position:
        driveBack()
