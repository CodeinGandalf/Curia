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
from std_msgs.msg import Int8MultiArray
from simple_pid import PID

# Define the path to the folder of the subfiles:
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

import searchCanCam as scc


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
        rospy.loginfo("pressed rot front\r")
        drot += 0.15
    elif key == 'q':
        rospy.loginfo("pressed rot back\r")
        drot -= 0.15
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
def mecanum_inv_kinematics(vx, vy, omega, wheelRadius=0.044, L=0.250, W=0.132):
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


# Callback function to collect the wheel speeds traced by the encoders:
def wheel_speed_callback(msg):
    global current_wheel_speeds
    current_wheel_speeds = dict(zip(msg.name, msg.velocity))


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
        rospy.logerr("Error while trying to get the current pose.")
        return None


# Set the function for the motor pwm:
def set_motor_pwm(pca, channel_forward, channel_backward, pwm_value, MAX_PWM):
    # If the provided max PWM value is larger then the limit set it to the limit:
    limit = 65535*0.8

    if MAX_PWM > limit:
        MAX_PWM = limit
    elif -MAX_PWM > limit:
        MAX_PWM = -limit

    # Clip the PWM value if needed:
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
    # Work with 10 steps to smooth the servo movement:
    step_const = 5

    # Check what servo movement should be generated (four possible states cause there are 2 servos with 2 possible states):
    if pwm_value == 950:
        pwm = 1600
        step_size = -step_const
    elif pwm_value == 1600:
        pwm = 950
        step_size = step_const
    else:
        # If there is an other PWM value then known write it onto the pin without smoothing it:
        pi.set_servo_pulsewidth(Pin, pwm_value)
        return

    # Calculate the step size for the 10 steps:
    steps = int((pwm_value - pwm) / step_size)

    # Process 9 steps and then set the PWM value to the requested PWM value:
    for m in range(steps - 1):
        pwm = int(pwm + step_size)
        pi.set_servo_pulsewidth(Pin, pwm)
        rospy.sleep(0.01)

    pi.set_servo_pulsewidth(Pin, pwm_value)
    

# Callback for map updates:
def map_callback(msg):
    global latest_map
    latest_map = copy.deepcopy(msg)


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

    rospy.loginfo(f'shape map 1: {map1_array.shape}\r')
    rospy.loginfo(f'shape map 2: {map2_array.shape}\r')

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
    rospy.loginfo(f"Targets: {num_features}\r")

    # Setup the arrays to save the positions:
    world_x = []
    world_y = []
    area = []

    # Get the area and the CoG:
    for i in range(1, num_features + 1):
        region = (labeled_mask == i)
        area.append(np.sum(region)*resolution**2*10000)
        centroid = center_of_mass(region)

        # Calculate the pose in the global coordinate system:
        y_idx, x_idx = centroid
        world_x.append(min_origin_x + (final_width - x_idx) * resolution)
        world_y.append(min_origin_y + (final_height - y_idx) * resolution)

        rospy.loginfo(f"Targets {i}\r")
        rospy.loginfo(f"Area: {area[i - 1]}\r")
        rospy.loginfo(f"CoG: x = {world_x[i - 1]:.2f}, y = {world_y[i - 1]:.2f}\r")
    return list(zip(world_x, world_y, area))


# Function to convert the PID controller value to an PWM value:
def pid_output_to_pwm(corr, v_max=3.0, pwm_max=65535*0.8):
    # Clamping the PID output to the interval of [0, v_max]
    corr_clamped = max(0, min(corr, v_max))
    
    # Scale the values up to the max PWM value:
    pwm_val = int((corr_clamped / v_max) * pwm_max)
    return pwm_val


# Function to update the PWM for the engines:
def driveEngines(wheel_speeds, trueSpeedFL, trueSpeedFR, trueSpeedBL, trueSpeedBR, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR):
    # Add the target variables:
    target_FL = wheel_speeds[0, 0]
    target_FR = wheel_speeds[1, 0]
    target_BL = wheel_speeds[2, 0]
    target_BR = wheel_speeds[3, 0]

    max_speed = 3
    max_pwm=65535*0.8

    """ # Update the setpoint and the constants for the controller:
    pid_FL = PID(0.5, 0.1, 0.02, setpoint=target_FL)
    pid_FR = PID(0.5, 0.1, 0.02, setpoint=target_FR)
    pid_BL = PID(0.5, 0.1, 0.02, setpoint=target_BL)
    pid_BR = PID(0.5, 0.1, 0.02, setpoint=target_BR)

    # Calculate the correction for the engine:
    corr_FL = pid_FL(trueSpeedFL)
    corr_FR = pid_FR(trueSpeedFR)
    corr_BL = pid_BL(trueSpeedBL)
    corr_BR = pid_BR(trueSpeedBR)

    # Calculte the correct PWM values from the PID controller:
    pwm_fr = pid_output_to_pwm(corr_FR)
    pwm_fl = pid_output_to_pwm(corr_FL)
    pwm_bl = pid_output_to_pwm(corr_BL)
    pwm_br = pid_output_to_pwm(corr_BR)"""

    pwm_fl = target_FL*max_pwm/max_speed
    pwm_bl = target_BL*max_pwm/max_speed
    pwm_fr = target_FR*max_pwm/max_speed
    pwm_br = target_BR*max_pwm/max_speed

    # Print the true speed and target speed for all wheels:
    # rospy.loginfo(f'True Speed FL: {trueSpeedFL:.3f}, Target: {wheel_speeds[0, 0]:.3f}\r')
    # rospy.loginfo(f'True Speed FR: {trueSpeedFR:.3f}, Target: {wheel_speeds[1, 0]:.3f}\r')
    # rospy.loginfo(f'True Speed BL: {trueSpeedBL:.3f}, Target: {wheel_speeds[2, 0]:.3f}\r')
    # rospy.loginfo(f'True Speed BR: {trueSpeedBR:.3f}, Target: {wheel_speeds[3, 0]:.3f}\r')

    # Check if one of the engines has reached the max PWM value:
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


# Define the main function:
def main():
    # Init the node and subscriber:
    rospy.init_node('dose_sammler')
    # Subscriber starten
    rospy.Subscriber("/wheel_speeds", JointState, wheel_speed_callback)
    rospy.Subscriber('/keyboard_input', String, key_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    pub = rospy.Publisher('/wheel_directions', Int8MultiArray, queue_size=10)

    # Define all global variables as global:
    global take_map1, take_map2, dx, dy, drot, dGripper, dElevator, poseCanManual, homePose, run, latest_map, current_wheel_speeds

    # Define the channels for the engines and the servos:
    MOTOR_FL = (0, 1)
    MOTOR_FR = (2, 3)
    MOTOR_BL = (4, 5)
    MOTOR_BR = (6, 7)
    PWM_PIN_GRIPPER = 18
    PWM_PIN_ELEVATOR = 10

    msg = Int8MultiArray()
    msg.data = [1, 1, 1, 1]
    pub.publish(msg)    

    # Define the max PWM for the engines:
    MAX_PWM = 65535*0.8
    
    # Define the PWM values for the servos:
    GRIPPER_OPEN = 1600
    GRIPPER_CLOSED = 950
    ELEVATOR_BOTTOM = 2400
    ELEVATOR_TOP = 1318
    
    # Initalise the PWM values for the servos:
    pwm_gripper = GRIPPER_OPEN
    pwm_elevator = ELEVATOR_BOTTOM

    # Initalise the variable for the current_wheel_speeds in case there aren't any encoder data available:
    current_wheel_speeds = {}
    
    # Initialise the variable for the latest map as none, in case the setup for the map fails:
    latest_map = None

    # Set the variables to store the map to none: 
    map1 = None
    map2 = None
    
    # Set the variables to take a snap shot of the variables to none:
    take_map1 = False
    take_map2 = False

    # Initalise the variables for the movement to 0:
    dx = dy = drot = 0

    # Set the variables to control the servos with the keyboard to default:
    dGripper = pwm_gripper
    dElevator = pwm_elevator

    # Set the bools to check if the pose of the home pose / can pose should be computed to false as default:
    homePose = False
    poseCanManual = False

    # Define the needed variables to control the robot in the manual mode:
    old_dx = 0
    old_dy = 0
    old_drot = 0

    # Set the variable for the home pose and the pose of the can to none as default:
    poseOrigin = None
    poseCanWorld = None

    # Define a constante to calculate a speed target from the distance to drive the robot in the autonome mode to it's target:
    kp = 0.2

    # Set the bools to check if there is a can in the map data / if there really is a can to false as default:
    posCans = False
    isCan = False

    # Define the offset of the cam and the LIDAR to the base link:
    offsetCam = 87.98
    offsetLidar = 164.07

    # Define the index of the port for the USB cam:
    camera_index = '/dev/video0'

    # Set the bool to run the manual steering of the robot to true:
    run = True

    # Setup pigpio for the PWM pins to control the servos:
    pi = pigpio.pi()
    if not pi.connected:
        exit()

    # Initalise the I2C:
    i2c = busio.I2C(SCL, SDA)

    # Initalise the PWM board to control the engines, 1000 Hz is the max PWM value for that board:
    pca = PCA9685(i2c)
    pca.frequency = 1000

    # Set the frequency and PWM values for the servos:
    pi.set_PWM_frequency(PWM_PIN_GRIPPER, 50)
    pi.set_PWM_frequency(PWM_PIN_ELEVATOR, 50)
    pi.set_servo_pulsewidth(PWM_PIN_GRIPPER, GRIPPER_OPEN)
    pi.set_servo_pulsewidth(PWM_PIN_ELEVATOR, ELEVATOR_BOTTOM)

    # Shut down all channels on the pca board for the start:
    for channel in range(16):
        pca.channels[channel].duty_cycle = 0

    # Calculate the wheel_speeds with the default values:
    wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

    sign_FL = int(np.sign(wheel_speeds[0, 0]))
    sign_FR = int(np.sign(wheel_speeds[1, 0]))
    sign_BL = int(np.sign(wheel_speeds[2, 0]))
    sign_BR = int(np.sign(wheel_speeds[3, 0]))

    old_sign_FL = np.sign(wheel_speeds[0, 0])
    old_sign_FR = np.sign(wheel_speeds[1, 0])
    old_sign_BL = np.sign(wheel_speeds[2, 0])
    old_sign_BR = np.sign(wheel_speeds[3, 0])

    # Set the rate to 4 Hz:
    rate = rospy.Rate(4)

    # Drive manual through the room to generate the map:
    while run == True:
        # If the home Position has been selected get the current position as poseOrigin and set the speed targets to 0 / stop the loop:
        if homePose:
            rospy.loginfo("Get home pose\r")
            run = False
            poseOrigin = get_pose()
            rospy.loginfo(f'home pose: {poseOrigin}\r')
            homePose = False
            dx = 0
            dy = 0
            drot = 0
        
        # Get the pose of the can manualy:
        if poseCanManual == True:
            rospy.loginfo("Get can pose\r")
            poseCanWorld = get_pose()
            rospy.loginfo(f'Pose: can {poseCanWorld}\r')
            poseCanManual = False
        
        # Take a first snap shot of the map without a can in the room:
        if take_map1 is True:
            rospy.loginfo("Taking the snap shot for the first map\r")
            rospy.loginfo(f'latest map: {latest_map}\r')
            map1 = copy.deepcopy(latest_map)
            rospy.loginfo("Snapshot 1\r")
            take_map1 = False

        # Take a second snap shot of the map, now with the can in the room:
        if take_map2 is True:
            # Check if there is a snap shot of the first map without a can:
            if map1 is None:
                rospy.logwarn("You haven't generated a first snap shot without the can on the map.")
                take_map2 = False
            else:
                map2 = copy.deepcopy(latest_map)
                rospy.loginfo("Snapshot 2\r")
                take_map2 = False
                run = False
                dx = 0
                dy = 0
                drot = 0
                poseOrigin = get_pose()

        # Calculate the difference to the value before:
        diff_dx = abs(dx - old_dx)
        diff_dy = abs(dy - old_dy)
        diff_drot = abs(drot - old_drot)

        # If there was a change in the value update the wheel speeds:
        if diff_dx != 0 or diff_dy != 0 or diff_drot != 0 or dx == 0 or dy == 0 or drot == 0:
            rospy.loginfo(f'Val drot: {drot}\r')
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)
        
        # Update the PWM values for the servos only if changed:
        if pwm_gripper != dGripper:
            pwm_gripper = dGripper
            set_servo_pwm(pi, PWM_PIN_GRIPPER, pwm_gripper)
        if pwm_elevator != dElevator:
            pwm_elevator = dElevator
            set_servo_pwm(pi, PWM_PIN_ELEVATOR, pwm_elevator)

        # Current motor speeds calculated with the encoder data in rad/s:
        trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
        trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
        trueSpeed_BL = current_wheel_speeds.get('BL', 0.0)
        trueSpeed_BR = current_wheel_speeds.get('BR', 0.0)


        sign_FL = int(np.sign(wheel_speeds[0, 0]))
        sign_FR = int(np.sign(wheel_speeds[1, 0]))
        sign_BL = int(np.sign(wheel_speeds[2, 0]))
        sign_BR = int(np.sign(wheel_speeds[3, 0]))

        new_signs = [sign_FL, sign_FR, sign_BL, sign_BR]
        old_signs = [old_sign_FL, old_sign_FR, old_sign_BL, old_sign_BR]

        if new_signs != old_signs:
            msg.data = [sign_FL, sign_FR, sign_BL, sign_BR]
            pub.publish(msg)

        old_sign_FL = sign_FL
        old_sign_FR = sign_FR
        old_sign_BL = sign_BL
        old_sign_BR = sign_BR

        # Update the engine targets:
        driveEngines(wheel_speeds, trueSpeed_FL, trueSpeed_FR, trueSpeed_BL, trueSpeed_BR, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

        rate.sleep()

    # Set the engine PWM targets to 0:
    set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], 0, MAX_PWM)
    set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], 0, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], 0, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], 0, MAX_PWM)

    # Check if there are 2 arrays with some map data in it:
    if map1 is not None and map2 is not None:
        # Calculate the pose of the can:
        pose = calculatePoseCan(map1, map2)

        # Define positionCan as an array:
        positionCan = []

        # Print all contours in the mask:
        for i, (x, y, a) in enumerate(pose):
            rospy.loginfo(f"Object {i + 1}: x = {x:.2f}, y = {y:.2f}, Area = {a:.3f} cm²\r")

            if a >= 36.0 and a <= 64.0:
                rospy.loginfo(f'Pose can: {x:.2f}, {y:.2f}\r')
                positionCan.append((x, y))
                posCans = True
    else:
        rospy.loginfo("map1 and or map2 are / is empty\r")
    
    # Setup the pins for the Leuze sensors:
    PINLEUZE1 = 25
    PINLEUZE2 = 9
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PINLEUZE1, GPIO.IN)
    GPIO.setup(PINLEUZE2, GPIO.IN)

    # Read the current state:
    Leuze1 = GPIO.input(PINLEUZE1)
    Leuze2 = GPIO.input(PINLEUZE2)

    # Check if there is a can detected in the map data:
    if posCans:
        # Calculate the difference in the pose of the can and the current pose:
        target_pose_x = positionCan[0][0]
        target_pose_y = positionCan[0][1]
        diff_pose_x = target_pose_x - poseOrigin.pose.x
        diff_pose_y = target_pose_y - poseOrigin.pose.y

        # Correct the y offset:
        while diff_pose_y > 0.05:
            # Calculate the value of dy to drive sideways:
            dx = 0
            dy = diff_pose_y*kp
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Current motor speeds in rad/s:
            trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
            trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
            trueSpeed_BL = current_wheel_speeds.get('BL', 0.0)
            trueSpeed_BR = current_wheel_speeds.get('BR', 0.0)

            sign_FL = int(np.sign(wheel_speeds[0, 0]))
            sign_FR = int(np.sign(wheel_speeds[1, 0]))
            sign_BL = int(np.sign(wheel_speeds[2, 0]))
            sign_BR = int(np.sign(wheel_speeds[3, 0]))

            new_signs = [sign_FL, sign_FR, sign_BL, sign_BR]
            old_signs = [old_sign_FL, old_sign_FR, old_sign_BL, old_sign_BR]

            if new_signs != old_signs:
                msg.data = [sign_FL, sign_FR, sign_BL, sign_BR]
                pub.publish(msg)

            old_sign_FL = sign_FL
            old_sign_FR = sign_FR
            old_sign_BL = sign_BL
            old_sign_BR = sign_BR

            # Update the PWM targets for the eninges:
            driveEngines(wheel_speeds, trueSpeed_FL, trueSpeed_FR, trueSpeed_BL, trueSpeed_BR, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the current pose:
            currentPose = get_pose()

            # Calculate the difference in the pose:
            diff_pose_x = target_pose_x - currentPose.pose.x
            diff_pose_y = target_pose_y - currentPose.pose.y

            rate.sleep()
        
        # Set the engine targets to 0:
        set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], 0, MAX_PWM)
        set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], 0, MAX_PWM)
        set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], 0, MAX_PWM)
        set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], 0, MAX_PWM)

        # Drive towards the can:
        while diff_pose_x > 0.5:
            # Update the dx and dx target:
            dx = diff_pose_x*kp
            dy = 0
            drot = 0

            # Calculate the new wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Current motor speeds in rad/s:
            trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
            trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
            trueSpeed_BL = current_wheel_speeds.get('BL', 0.0)
            trueSpeed_BR = current_wheel_speeds.get('BR', 0.0)

            sign_FL = int(np.sign(wheel_speeds[0, 0]))
            sign_FR = int(np.sign(wheel_speeds[1, 0]))
            sign_BL = int(np.sign(wheel_speeds[2, 0]))
            sign_BR = int(np.sign(wheel_speeds[3, 0]))

            new_signs = [sign_FL, sign_FR, sign_BL, sign_BR]
            old_signs = [old_sign_FL, old_sign_FR, old_sign_BL, old_sign_BR]

            if new_signs != old_signs:
                msg.data = [sign_FL, sign_FR, sign_BL, sign_BR]
                pub.publish(msg)

            old_sign_FL = sign_FL
            old_sign_FR = sign_FR
            old_sign_BL = sign_BL
            old_sign_BR = sign_BR

            # Update the engine targets:
            driveEngines(wheel_speeds, trueSpeed_FL, trueSpeed_FR, trueSpeed_BL, trueSpeed_BR, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the current pose:
            currentPose = get_pose()

            # Calculate the difference between the current pose and the target:
            diff_pose_x = target_pose_x - currentPose.pose.x
            diff_pose_y = target_pose_y - currentPose.pose.y

            rate.sleep()

        # Check if there is a can based upon the LIDAR and cam data:
        # Use the LIDAR data and search for the can in the picture:
        isCan = scc.find_best_can(camera_index)

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

                # Current motor speeds in rad/s:
                trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
                trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
                trueSpeed_BL = current_wheel_speeds.get('BL', 0.0)
                trueSpeed_BR = current_wheel_speeds.get('BR', 0.0)

                sign_FL = int(np.sign(wheel_speeds[0, 0]))
                sign_FR = int(np.sign(wheel_speeds[1, 0]))
                sign_BL = int(np.sign(wheel_speeds[2, 0]))
                sign_BR = int(np.sign(wheel_speeds[3, 0]))

                new_signs = [sign_FL, sign_FR, sign_BL, sign_BR]
                old_signs = [old_sign_FL, old_sign_FR, old_sign_BL, old_sign_BR]

                if new_signs != old_signs:
                    msg.data = [sign_FL, sign_FR, sign_BL, sign_BR]
                    pub.publish(msg)

                old_sign_FL = sign_FL
                old_sign_FR = sign_FR
                old_sign_BL = sign_BL
                old_sign_BR = sign_BR

                # Update the engine PWM targets:
                driveEngines(wheel_speeds, trueSpeed_FL, trueSpeed_FR, trueSpeed_BL, trueSpeed_BR, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

                # Read from the Leuze sensors if they can see the can:
                Leuze1 = GPIO.input(PINLEUZE1)
                Leuze2 = GPIO.input(PINLEUZE2)

                rate.sleep()

            # Update the PMW value for the engines and the servos:
            set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], 0, MAX_PWM)
            set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], 0, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], 0, MAX_PWM)
            set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], 0, MAX_PWM)

            # Set the PWM values for the servos to grab the can:
            pwm_gripper = GRIPPER_CLOSED
            pwm_elevator = ELEVATOR_TOP

            # Update the PWM for the gripper to grab the can:
            set_servo_pwm(pi, PWM_PIN_GRIPPER, pwm_gripper)

            # Update the elevator PWM value to lift the can:
            set_servo_pwm(pi, PWM_PIN_ELEVATOR, pwm_elevator)

        # Get the current pose:
        pose = get_pose()

        # Calculate the offset from the current pose and the home pose:
        pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
        pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y

        # While this offset is to big update the PWM values to get closer to the home pose:
        while abs(pose_offset_x) > 0.05 and abs(pose_offset_y) > 0.05:
            # Get the new pose:
            pose = get_pose()

            # Calculate the new offset:
            pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
            pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y

            # Calculate the new speed targets:
            dx = pose_offset_x*kp
            dy = pose_offset_y*kp
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Current motor speeds in rad/s:
            trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
            trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
            trueSpeed_BL = current_wheel_speeds.get('BL', 0.0)
            trueSpeed_BR = current_wheel_speeds.get('BR', 0.0)

            sign_FL = int(np.sign(wheel_speeds[0, 0]))
            sign_FR = int(np.sign(wheel_speeds[1, 0]))
            sign_BL = int(np.sign(wheel_speeds[2, 0]))
            sign_BR = int(np.sign(wheel_speeds[3, 0]))

            new_signs = [sign_FL, sign_FR, sign_BL, sign_BR]
            old_signs = [old_sign_FL, old_sign_FR, old_sign_BL, old_sign_BR]

            if new_signs != old_signs:
                msg.data = [sign_FL, sign_FR, sign_BL, sign_BR]
                pub.publish(msg)

            old_sign_FL = sign_FL
            old_sign_FR = sign_FR
            old_sign_BL = sign_BL
            old_sign_BR = sign_BR
            
            # Update the engine targets:
            driveEngines(wheel_speeds, trueSpeed_FL, trueSpeed_FR, trueSpeed_BL, trueSpeed_BR, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            rate.sleep()
        
        # Set the elevator PWM value to the bottom state:
        pwm_elevator = ELEVATOR_BOTTOM

        # Update the PWM value for the elevator to lower the can:
        set_servo_pwm(pi, PWM_PIN_ELEVATOR, pwm_elevator)

        # Set the gripper PWM value to the open state:
        pwm_gripper = GRIPPER_OPEN

        # Update the PWM value for the gripper to open the gripper:
        set_servo_pwm(pi, PWM_PIN_GRIPPER, pwm_gripper)

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
