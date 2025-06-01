#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy
import matplotlib.pyplot as plt
import os
import sys
import threading
import smbus
import subprocess
import time
import math
import numpy as np
from std_msgs.msg import String
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import tf
from geometry_msgs.msg import PoseStamped
import RPi.GPIO as GPIO
import pigpio
import matplotlib.pyplot as plt
import copy
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import label, center_of_mass
from sensor_msgs.msg import JointState

sys.path.append(os.path.join(os.path.dirname(__file__), '../scripts'))

import lidar_subscriber

# Define the path to the folder of the subfiles:
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

import searchCan as SC
import checkIfCan as CIC

# Globale Variable, um auf die Daten zuzugreifen
current_wheel_speeds = {}


# Def callback function for the keys:
def key_callback(msg):
    # Define the global variables as global:
    global dx, dy, drot, run, dGripper, dElevator, homePose, take_map1, take_map2, poseCanManual
    key = msg.data.lower()

    # Define the knows keys:
    if key == 'w':
        dx += 0.02
    elif key == 's':
        dx -= 0.02
    elif key == 'a':
        dy -= 0.02
    elif key == 'd':
        dy += 0.02
    elif key == 'e':
        drot += 0.04
    elif key == 'q':
        drot -= 0.04
    elif key == 'f':
        dElevator = 2400
    elif key == 'r':
        dElevator = 1318
    elif key == 't':
        dGripper = 1600
    elif key == 'g':
        dGripper = 1100
    elif key == 'p':
        dx = 0
        dy = 0
        drot = 0
    elif key == 'h':
        homePose = True
    elif key == 'j':
        poseCanManual = True
    elif key == 'n':
        take_map1 = True
        rospy.loginfo("take map 1\r")
    elif key == 'm':
        take_map2 = True
        rospy.loginfo("Take map 2\r")
    elif key == '0':
        dx = 0
        dy = 0
        drot = 0
        run = False
        rospy.loginfo("Escape\r")


# Def func for inv kinematics
def mecanum_inv_kinematics(vx, vy, omega, wheelRadius=0.044, L=0.244, W=0.132):
    # Matrix for the inverse kinematics:
    J_inv = 1/wheelRadius*np.matrix([
        [1, -1, -(L + W)],
        [1, 1, (L + W)],
        [1, 1, -(L + W)],
        [1, -1, (L + W)]
    ])

    # Define the matrix with the velocitys:
    velocityVector = np.matrix([vx, vy, omega]).T
    wheelSpeeds = J_inv * velocityVector

    # Return the wheel speeds:
    return wheelSpeeds.reshape(-1, 1)


def wheel_speed_callback(msg):
    global current_wheel_speeds
    # msg.name ist eine Liste von Strings
    # msg.velocity ist eine Liste von floats
    current_wheel_speeds = dict(zip(msg.name, msg.velocity))


def get_LIDAR_Data():
    scan = lidar_subscriber.get_latest_scan()

    max_range = 5.0
    threshold_distance = 0.15
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    angle_distance_array = []

    def normalize_angle(angle_rad):
        return (angle_rad + np.pi) % (2 * np.pi) - np.pi

    def in_angle_range(angle, min_angle, max_angle):
        if min_angle < max_angle:
            return min_angle <= angle <= max_angle
        else:
            return angle >= min_angle or angle <= max_angle

    filter_angle_min = normalize_angle(np.deg2rad(308))
    filter_angle_max = normalize_angle(np.deg2rad(330))

    for i, distance in enumerate(scan.ranges):
        angle = normalize_angle(angle_min + i * angle_increment)

        if not (0.0 < distance < max_range):
            distance = max_range

        if in_angle_range(angle, filter_angle_min, filter_angle_max):
            distance = max_range

        angle_distance_array.append((distance, angle))
        #rospy.loginfo(f"{np.rad2deg(angle):.1f}°, final={distance:.2f}m")

    return np.array(angle_distance_array)



# Get the current pose:
def get_pose():
    # Get the transformlistener:
    listener = tf.TransformListener()
    rospy.sleep(1.0)

    # Try to get the current pose:
    try:
        # Wait for the transformation:
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))

        # Get the translation and rotation in the global coordinate system:
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))

        # Update the pose:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]

        return pose
    except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        rospy.logerr("Fehler beim Auslesen der Position.")
        return None


# Set the function for the motor pwm:
def set_motor_pwm(pca, channel_forward, channel_backward, pwm_value, MAX_PWM):
    pwm_value = int(np.clip(pwm_value, -MAX_PWM, MAX_PWM))

    # Check if the engine turns forward or backwards:
    if pwm_value >= 0:
        pca.channels[channel_forward].duty_cycle = 0
        pca.channels[channel_backward].duty_cycle = pwm_value
    else:
        pca.channels[channel_forward].duty_cycle = -pwm_value
        pca.channels[channel_backward].duty_cycle = 0


# Define the function for the servo pwm:
def set_servo_pwm(pi, Pin, pwm_value):
    pi.set_servo_pulsewidth(Pin, pwm_value)


# Callback for map updates:
def map_callback(msg):
    rospy.logwarn("Update map Callback")
    global latest_map
    latest_map = copy.deepcopy(msg)
   # rospy.loginfo(f'latest map: {latest_map}')


# Generate the array out of the map message data:
def map_to_array(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
    return data


# Crop the map from the sides:
def crop_map(map_array, margin):
    return map_array[margin:-margin, margin:-margin]


# Get the pose of the can:
def calculatePoseCan(map1, map2):
    # Get the resolution and define the margin:
    resolution = map1.info.resolution
    margin = 0.3

    # Calculate the amount of cells:
    crop_cells = int(margin / resolution)

    # Calculate the arrays for the maps:
    map1_array = map_to_array(map1)
    map2_array = map_to_array(map2)

    # Crop the margin for the maps:
    map1_cropped = crop_map(map1_array, crop_cells)
    map2_cropped = crop_map(map2_array, crop_cells)

    # Get the old origin coords:
    old_origin_x1 = map1.info.origin.position.x
    old_origin_y1 = map1.info.origin.position.y
    old_origin_x2 = map2.info.origin.position.x
    old_origin_y2 = map2.info.origin.position.y

    # Calculate the new coords after the croping:
    new_origin_x1 = old_origin_x1 + margin
    new_origin_y1 = old_origin_y1 + margin
    new_origin_x2 = old_origin_x2 + margin
    new_origin_y2 = old_origin_y2 + margin

    # Calculate the diff of the origins:
    diff_x = new_origin_x2 - new_origin_x1
    diff_y = new_origin_y2 - new_origin_y1

    # Get the min value for the origins:
    min_origin_x = min(new_origin_x1, new_origin_x2)
    min_origin_y = min(new_origin_y1, new_origin_y2)

    # Check if there is an offset of the origins:
    if diff_x != 0 or diff_y != 0:
        offset_x1 = abs(new_origin_x1 - min_origin_x)
        offset_y1 = abs(new_origin_y1 - min_origin_y)
        offset_x2 = abs(new_origin_x2 - min_origin_x)
        offset_y2 = abs(new_origin_y2 - min_origin_y)

        # Calculate the amount of cells of the offset:
        offset_cells_x1 = int(round(offset_x1 / resolution))
        offset_cells_y1 = int(round(offset_y1 / resolution))
        offset_cells_x2 = int(round(offset_x2 / resolution))
        offset_cells_y2 = int(round(offset_y2 / resolution))

        # Crop the maps to bring the origins together:
        map1_cropped = map1_cropped[:-offset_cells_y1, offset_cells_x1:]
        map2_cropped = map2_cropped[:-offset_cells_y2, offset_cells_x2:]

    # Calculate the new width and height of the maps:
    width_map1 = map1_cropped.shape[1]
    height_map1 = map1_cropped.shape[0]
    width_map2 = map2_cropped.shape[1]
    height_map2 = map2_cropped.shape[0]

    # Check if both maps have the same width and height:
    diff_width = abs(width_map2 - width_map1)
    diff_height = abs(height_map2 - height_map1)

    # Set the min width and height:
    final_width = min(map1_cropped.shape[1], map2_cropped.shape[1])
    final_height = min(map1_cropped.shape[0], map2_cropped.shape[0])

    # If the maps have a different shape, correct it:
    if diff_width != 0 or diff_height != 0:
        # Crop the maps to get them to the same shape:
        map1_cropped = map1_cropped[-final_height:, :final_width]
        map2_cropped = map2_cropped[-final_height:, :final_width]

    # Now the maps should have the same origin and the same shape; calculate the difference in the maps:
    mask = map2_cropped > map1_cropped

    # Serch for the positions that belong together:
    structure = np.ones((3, 3), dtype=int)
    labeled_mask, num_features = label(mask, structure=structure)
    rospy.loginfo(f"Targets: {num_features}")

    # Setup the arrays to save the positions:
    world_x = []
    world_y = []
    area = []

    # Get the area and the CoG:
    for i in range(1, num_features + 1):
        region = (labeled_mask == i)
        area.append(np.sum(region)*resolution**2)
        centroid = center_of_mass(region)

        # Calculate the pose in the global coordinate system:
        y_idx, x_idx = centroid
        world_x.append(min_origin_x + (final_width - x_idx) * resolution)
        world_y.append(min_origin_y + (final_height - y_idx) * resolution)

        rospy.loginfo(f"Targets {i}:")
        rospy.loginfo(f"Area: {area[i - 1]}")
        rospy.loginfo(f"CoG: x = {world_x[i - 1]:.2f}, y = {world_y[i - 1]:.2f}")
    return list(zip(world_x, world_y, area))


# Define the main function:
def main():
    # Init the node and subscriber:
    rospy.init_node('dose_sammler')
    # Subscriber starten
    rospy.Subscriber("/wheel_states", JointState, wheel_speed_callback)
    rospy.Subscriber('/keyboard_input', String, key_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    rospy.sleep(2)

    # Close all plots if any plots are opened:
    plt.close('all')

    # Initalise the I2C:
    i2c = busio.I2C(SCL, SDA)

    # Initalise the PWM board:
    pca = PCA9685(i2c)
    pca.frequency = 1000

    # Define the channels for the engines and the servos:
    MOTOR_FL = (0, 1)
    MOTOR_FR = (2, 3)
    MOTOR_BL = (4, 5)
    MOTOR_BR = (6, 7)
    
    PWM_PIN_GRIPPER = 18
    PWM_PIN_ELEVATOR = 10
    
    pi = pigpio.pi()
    if not pi.connected:
        exit()
    
    # Define the PWM values for the servos:
    GRIPPER_OPEN = 1600
    GRIPPER_CLOSED = 1100
    ELEVATOR_BOTTOM = 2400
    ELEVATOR_TOP = 1318
    
    pi.set_PWM_frequency(PWM_PIN_GRIPPER, 50)
    rospy.sleep(1)
    pi.set_PWM_frequency(PWM_PIN_ELEVATOR, 50)
    
    pi.set_servo_pulsewidth(PWM_PIN_GRIPPER, GRIPPER_OPEN)
    rospy.sleep(1)
    pi.set_servo_pulsewidth(PWM_PIN_ELEVATOR, ELEVATOR_BOTTOM)
    rospy.sleep(1)

    # Define the max PWM:
    MAX_PWM = 65535*0.8

    # Initalise the PWM values:
    pwm_fl = 0
    pwm_fr = 0
    pwm_bl = 0
    pwm_br = 0
    pwm_gripper = GRIPPER_OPEN
    pwm_elevator = ELEVATOR_BOTTOM
    
    global take_map1, take_map2, dx, dy, drot, dGripper, dElevator, poseCanManual, homePose, run, latest_map
    
    latest_map = None
        
    map1 = None
    map2 = None
    
    take_map1 = False
    take_map2 = False
    dx = dy = drot = dGripper = dElevator = 0
    homePose = False
    poseCanManual = False
    run = True

    # Shut down all channels on the pca board:
    for channel in range(16):
        pca.channels[channel].duty_cycle = 0
    
    # Set the PWM values to default:
    set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
    set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)

    # Define the needed variables:
    dx = 0
    dy = 0
    drot = 0
    old_dx = 0
    old_dy = 0
    old_drot = 0
    MAX_SPEED = 3
    poseOrigin = None
    poseCanWorld = None
    kp = 0.2

    # Calculate the wheel_speeds with the default values:
    wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

    # Start the LIADR node:
    lidar_subscriber.init_lidar_subscriber()

    rospy.sleep(1)

    # Get the first LIDAR data:
    lidarData = get_LIDAR_Data()

    # Print the LIDAR data if there are any LIDAR data:
    if lidarData is not None and lidarData.size > 0:
        rospy.loginfo("\rtrying to print the data \r")
        rospy.loginfo(f'lidardata: {lidarData[0][0]:.3f} m\r')
    else:
        rospy.logwarn("\rNoch keine LIDAR-Daten empfangen.\r")

    rospy.sleep(1)

    # Set the rate to 0.5Hz; 2s:
    rate = rospy.Rate(4)

    # Drive manual through the room to generate the map:
    while run == True:
        # If the home Position has been selected get the current position as home pose:
        if homePose:
            rospy.loginfo("Get home pose\r")
            run = False
            poseOrigin = get_pose()
            rospy.loginfo(f'home pose: {poseOrigin}\r')
            homePose = False
        
        if poseCanManual == True:
            rospy.loginfo("Get can pose\r")
            poseCanWorld = get_pose()
            rospy.loginfo(f'Pose: can {poseCanWorld}\r')
            poseCanManual = False
        
        if take_map1 is True:
            rospy.loginfo("Taking the snap shot for the first map")
            rospy.loginfo(f'latest map: {latest_map}')
            map1 = copy.deepcopy(latest_map)
            rospy.loginfo("Snapshot 1\r")
            take_map1 = False

        if take_map2 is True:
            map2 = copy.deepcopy(latest_map)
            rospy.loginfo("Snapshot 2\r")
            take_map2 = False
            run = False
            dx = 0
            dy = 0
            drot = 0

        # Print the current speeds:
        rospy.loginfo(f"dx: {dx} m/s | dy: {dy} m/s | drot: {drot} m/s\r")

        # Calculate the difference to the value before:
        diff_dx = abs(dx - old_dx)
        diff_dy = abs(dy - old_dy)
        diff_drot = abs(drot - old_drot)

        # If there was a change in the value update the wheel speeds:
        if diff_dx != 0 or diff_dy != 0 or diff_drot != 0 or dx == 0 or dy == 0 or drot == 0:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)
        
        pwm_gripper = dGripper
        pwm_elevator = dElevator
        
        # Target variable im Loop hinzufuegen:
        target_FL = wheel_speeds[0, 0]
        target_FR = wheel_speeds[1, 0]
        target_BL = wheel_speeds[2, 0]
        target_BR = wheel_speeds[3, 0]

        # Current motor speeds in rad/s
        trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
        trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
        trueSpeed_BL = current_wheel_speeds.get('BL', 0.0)
        trueSpeed_BR = current_wheel_speeds.get('BR', 0.0)

        dv_FL = (target_FL - trueSpeed_FL)*kp + target_FL
        dv_FR = (target_FR - trueSpeed_FR)*kp + target_FR
        dv_BL = (target_BL - trueSpeed_BL)*kp + target_BL
        dv_BR = (target_BR - trueSpeed_BR)*kp + target_BR

        pwm_fl = dv_FL / MAX_SPEED * MAX_PWM
        pwm_fr = dv_FR / MAX_SPEED * MAX_PWM
        pwm_bl = dv_BL / MAX_SPEED * MAX_PWM
        pwm_br = dv_BR / MAX_SPEED * MAX_PWM

        # Print the engine speeds:
        # Print the true speed and target speed for all wheels
        rospy.loginfo(f'True Speed FL: {trueSpeed_FL:.3f}, Target: {wheel_speeds[0, 0]:.3f}\r')
        rospy.loginfo(f'True Speed FR: {trueSpeed_FR:.3f}, Target: {wheel_speeds[1, 0]:.3f}\r')
        rospy.loginfo(f'True Speed BL: {trueSpeed_BL:.3f}, Target: {wheel_speeds[2, 0]:.3f}\r')
        rospy.loginfo(f'True Speed BR: {trueSpeed_BR:.3f}, Target: {wheel_speeds[3, 0]:.3f}\r')

        if pwm_fl > MAX_PWM or pwm_fr > MAX_PWM or pwm_bl > MAX_PWM or pwm_br > MAX_PWM:
            if pwm_fl > MAX_PWM:
                # Update the PWM values for the engines and the servos:
                set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
                set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, pwm_fr)
                set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, pwm_bl)
                set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, pwm_br)
            elif pwm_fr > MAX_PWM:
                # Update the PWM values for the engines and the servos:
                set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, pwm_fl)
                set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
                set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, pwm_bl)
                set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, pwm_br)
            elif pwm_bl > MAX_PWM:
                # Update the PWM values for the engines and the servos:
                set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, pwm_fl)
                set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, pwm_fr)
                set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
                set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, pwm_br)
            elif pwm_br > MAX_PWM:
                # Update the PWM values for the engines and the servos:
                set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, pwm_fl)
                set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, pwm_fr)
                set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, pwm_bl)
                set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)
        else:
            # Update the PWM values for the engines and the servos:
            set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)
        set_servo_pwm(pi, PWM_PIN_GRIPPER, pwm_gripper)
        set_servo_pwm(pi, PWM_PIN_ELEVATOR, pwm_elevator)

        rate.sleep()

    if map1 is not None and map2 is not None:
        pose = calculatePoseCan(map1, map2)
        
        map1_array = map_to_array(map1)
        map2_array = map_to_array(map2)
        rospy.loginfo(f'shape map 1: {map1_array.shape}')
        rospy.loginfo(f'shape map 2: {map2_array.shape}')
        

        for i, (x, y, a) in enumerate(pose):
            rospy.loginfo(f"Hindernis {i + 1}: x = {x:.2f}, y = {y:.2f}, Fläche = {a:.3f} m²")
    else:
        rospy.loginfo("map1 and or map2 is empty")

    # Set the starting variables to default:
    posCans = False
    isCan = False
    offsetCam = 87.98
    offsetLidar = 164.07
    camera_index = '/dev/video0'

    # Get teh current LIDAR data:
    lidarData = get_LIDAR_Data()

    distances = lidarData[:, 0]
    angles = lidarData[:, 1]

    # Search for the pose of the potentional cans:
    # posCans = SC.searchCan(lidarData)

    # Print the pose to the console:
    # rospy.loginfo(f'Found potential can: {posCans}\r')

    # Setup the pins for the Leuze sensors:
    PINLEUZE1 = 25
    PINLEUZE2 = 9
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PINLEUZE1, GPIO.IN)
    GPIO.setup(PINLEUZE2, GPIO.IN)

    # Read the current state:
    Leuze1 = GPIO.input(PINLEUZE1)
    Leuze2 = GPIO.input(PINLEUZE2)
    
    rospy.loginfo(f"Leuze1: {Leuze1}")
    rospy.loginfo(f"Leuze2: {Leuze2}")

    """
    # Check if there are any cans in the LIDAR data:
    if posCans:
        # Safe the current distance and angle information into posCan:
        dist = posCans[0][0]
        angle = posCans[0][1]

        # Correct the x offset:
        while (dist*np.cos(angle)) > 0.05:
            # Calculate the value of dx:
            dy = dist*np.cos(angle)*kp
            dx = 0
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Calculate the engine speeds from the wheel speeds:
            fl = wheel_speeds[0, 0]*2
            fr = wheel_speeds[1, 0]*2
            bl = wheel_speeds[2, 0]*2
            br = wheel_speeds[3, 0]*2

            # Update the PWM values:
            pwm_fl = fl/MAX_SPEED*MAX_PWM
            pwm_fr = fr/MAX_SPEED*MAX_PWM
            pwm_bl = bl/MAX_SPEED*MAX_PWM
            pwm_br = br/MAX_SPEED*MAX_PWM

            # Print the engine speeds:
            rospy.loginfo(f"Engine Speeds:\r\nFront Left: {fl:.2f} 1/s\r\nFront Right: {fr:.2f} 1/s\r\nBack Left: {bl:.2f} 1/s\r\nBack Right: {br:.2f} 1/s\r")

            # Update the PWM values for the engines:
            set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)

            # Get the current LIDAR data to check how big dx is now:
            lidarData = get_LIDAR_Data()

            # Search for the can again:
            posCans = SC.searchCan(lidarData)

            # Get the new distance and angle of the center from the can:
            dist = posCans[0][0]
            angle = posCans[0][1]

        # Drive towards the can:
        while (dist*np.sin(angle)) > 0.5:
            # Update the dx and dy target:
            dy = dist*np.cos(angle)*kp
            dx = dist*np.sin(angle)*kp
            drot = 0

            # Calculate the new wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Calculate the eninge speeds from the wheel speeds:
            fl = wheel_speeds[0, 0]*2
            fr = wheel_speeds[1, 0]*2
            bl = wheel_speeds[2, 0]*2
            br = wheel_speeds[3, 0]*2

            # Calculate the PWM value for the engine speed:
            pwm_fl = fl/MAX_SPEED*MAX_PWM
            pwm_fr = fr/MAX_SPEED*MAX_PWM
            pwm_bl = bl/MAX_SPEED*MAX_PWM
            pwm_br = br/MAX_SPEED*MAX_PWM

            # Print the engine speeds:
            rospy.loginfo(f"Engine Speeds:\r\nFront Left: {fl:.2f} 1/s\r\nFront Right: {fr:.2f} 1/s\r\nBack Left: {bl:.2f} 1/s\r\nBack Right: {br:.2f} 1/s\r")

            # Update the PWM values:
            set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)

            # Get the current LIDAR data:
            lidarData = get_LIDAR_Data()

            # Serach for the pose of the can:
            posCans = SC.searchCan(lidarData)

            # Update the distance and angle for the pose of the can:
            dist = posCans[0][0]
            angle = posCans[0][1]

        # Check if there is a can based upon the LIDAR and cam data:
        isCan, offset = CIC.checkIfCan(posCans, camera_index, offsetCam, offsetLidar)

        # Print the distance and angle of the can:
        rospy.loginfo(f'isCan: {isCan}, offset: {offset}')

        # If there is a can; collect it:
        if isCan:
            # Check the Leuze sensors. When they see the can stop the engines:
            while Leuze1 is False and Leuze2 is False:
                # Drive towards the can; no dx correction and no rotation needed here:
                dx = 0.5
                dy = 0
                drot = 0

                # Update the wheel speeds:
                wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

                # Calculate the engine speeds from the wheel speeds:
                fl = wheel_speeds[0, 0]*2
                fr = wheel_speeds[1, 0]*2
                bl = wheel_speeds[2, 0]*2
                br = wheel_speeds[3, 0]*2

                # Calculate the PWM values for the enigines:
                pwm_fl = fl/MAX_SPEED*MAX_PWM
                pwm_fr = fr/MAX_SPEED*MAX_PWM
                pwm_bl = bl/MAX_SPEED*MAX_PWM
                pwm_br = br/MAX_SPEED*MAX_PWM

                # Print the current engine speeds:
                rospy.loginfo(f"Engine Speeds:\r\nFront Left: {fl:.2f} 1/s\r\nFront Right: {fr:.2f} 1/s\r\nBack Left: {bl:.2f} 1/s\r\nBack Right: {br:.2f} 1/s\r")

                # Update the PWM value:
                set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
                set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
                set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
                set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)

                # Read from the Leuze sensors if they can see the can:
                Leuze1 = GPIO.input(PINLEUZE1)
                Leuze2 = GPIO.input(PINLEUZE2)

            # Set the PWM values for the engines back to 0 to stop the robot and close the gripper:
            pwm_fl = 0
            pwm_fr = 0
            pwm_bl = 0
            pwm_br = 0
            pwm_gripper = GRIPPER_CLOSED
            pwm_elevator = ELEVATOR_BOTTOM

            # Print the engine speeds:
            rospy.loginfo(f"Engine Speeds:\r\nFront Left: {fl:.2f} 1/s\r\nFront Right: {fr:.2f} 1/s\r\nBack Left: {bl:.2f} 1/s\r\nBack Right: {br:.2f} 1/s\r")

            # Update the PMW value for the engines and the servos:
            set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)
            set_servo_pwm(gripper, pwm_gripper)
            set_servo_pwm(elevator, pwm_elevator)

            # Sleep for 1s that the engines and servos can be updated:
            rospy.sleep(1)

            # Update the elevator PWM value:
            pwm_elevator = ELEVATOR_TOP
            set_servo_pwm(elevator, pwm_elevator)

            # Sleep for 1s that the servo can be updated:
            rospy.sleep(1)

        # Get the current pose:
        pose = get_pose()

        # Calculate the offset from the current pose and the home pose:
        pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
        pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y

        # While this offset is to big update the PWM values to get closer to the home pose:
        while abs(pose_offset_x) > 0.05 and abs(pose_offset_y) > 0.05:
            # Calculate the new speed targets:
            dx = pose_offset_x*kp
            dy = pose_offset_y*kp
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Calculate the engine speeds from the wheel speeds:
            fl = wheel_speeds[0, 0]*2
            fr = wheel_speeds[1, 0]*2
            bl = wheel_speeds[2, 0]*2
            br = wheel_speeds[3, 0]*2

            # Update the PWM values:
            pwm_fl = fl/MAX_SPEED*MAX_PWM
            pwm_fr = fr/MAX_SPEED*MAX_PWM
            pwm_bl = bl/MAX_SPEED*MAX_PWM
            pwm_br = br/MAX_SPEED*MAX_PWM

            # Print the engine speeds to the console:
            rospy.loginfo(f"Engine Speeds:\r\nFront Left: {fl:.2f} 1/s\r\nFront Right: {fr:.2f} 1/s\r\nBack Left: {bl:.2f} 1/s\r\nBack Right: {br:.2f} 1/s\r")

            # Update the PWM values:
            set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)

            # Sleep for 0.5s:
            rospy.sleep(0.5)

            # Get the new pose:
            pose = get_pose()

            # Calculate the new offset:
            pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
            pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y
        
        # Update the PWM value for the elevator lower the can:
        pwm_elevator = ELEVATOR_BOTTOM
        set_servo_pwm(elevator, pwm_elevator)

        # Sleep for 1s that the servo can update it's value
        rospy.sleep(1)

        # Update the PWM value for the gripper to open the gripper:
        pwm_gripper = GRIPPER_OPEN
        set_servo_pwm(gripper, pwm_gripper)
        
        rospy.sleep(1)
    """
    # End of the program:
    rospy.loginfo("Program has finished.")
    rospy.signal_shutdown("User interrupt ends the program.")
    sys.exit(0)

if __name__ == '__main__':
    try:
        # Call the main loop:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'ROS has exited with the exception: {e}')
