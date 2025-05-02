# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy
import matplotlib.pyplot as plt

import LIDAR_fake_data as LIDAR_fd
import searchCan as SC
import checkIfCan as CIC
import correctRot as cR
import driveBack as dB
import driveDefPath as ddp
import collectLidarData as cLD
import mapping
import correctOffset as cO


def main():
    rospy.init_node('dose_sammler')

    plt.close('all')

    # Start to collect the LIDAR data:
    lidarData = cLD.collectLidarData()

    # Get the fake LIDAR data:
    lidarData = LIDAR_fd.fake_data()

    # Calculate the map based upon the LIDAR data:
    home = mapping.startMapping(lidarData)

    # Set the starting variables to default:
    posCans = False
    isCan = False
    offsetCam = 100
    offsetLidar = 100
    camera_index = 1

    # Search for the pose of the potentional cans:
    posCans = SC.searchCan(lidarData)
    rospy.loginfo(f'Found potential can: {posCans}')

    # Check if there are any cans in the LIDAR data:
    for dist, angle in posCans:
        # Safe the current distance and angle information into posCan:
        posCan = [[dist, angle]]

        # Drive towards the can:
        cR.correctRot(posCan)

        # Check if there is a can based upon the LIDAR and cam data:
        isCan, offset = CIC.checkIfCan(posCan, camera_index,
                                       offsetCam, offsetLidar)

        rospy.loginfo(f'isCan: {isCan}, offset: {offset}')

        # If there is a can; collect it:
        if isCan:
            # First check if there is an offset / correct it:
            if offset != 0:
                cO.correctOffset(offset)

            # Move the robot into the position to grab the can:
            ddp.driveDefPath(home)
        else:
            # If there is no can, drive back to the home position:
            dB.driveBack(home, can=False)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'ROS has exited with the exception: {e}')
