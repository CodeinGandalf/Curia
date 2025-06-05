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
from transforms3d.euler import quat2euler

# Define the path to the folder of the subfiles:
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

import searchCanCam as scc

# Define the PWM values for the servos:
global GRIPPER_OPEN, GRIPPER_CLOSED, ELEVATOR_BOTTOM, ELEVATOR_TOP
GRIPPER_OPEN = 1600
GRIPPER_CLOSED = 1000
ELEVATOR_BOTTOM = 2400
ELEVATOR_TOP = 1318

# Def callback function for the keys:
def key_callback(msg):
    # Set the used variables to global:
    global dx, dy, drot, run, dGripper, dElevator, homePose, take_map1, take_map2, poseCanManual

    # Lower the key inputs:
    key = msg.data.lower()

    # Define the knows keys:
    if key == 'w':
        dx += 0.02
    elif key == 's':
        dx -= 0.02
    elif key == 'a':
        dy += 0.02
    elif key == 'd':
        dy -= 0.02
    elif key == 'e':
        drot += 0.15
    elif key == 'q':
        drot -= 0.15
    elif key == 'f':
        dElevator = ELEVATOR_BOTTOM
    elif key == 'r':
        dElevator = ELEVATOR_TOP
    elif key == 't':
        dGripper = GRIPPER_OPEN
    elif key == 'g':
        dGripper = GRIPPER_CLOSED
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


# Def func for inv kinematics:
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
    #rospy.loginfo(wheelSpeeds.reshape(-1, 1))
    return wheelSpeeds.reshape(-1, 1)


# Callback function to collect the wheel speeds traced by the encoders:
def wheel_speed_callback(msg):
    # Set the variable to global to be able to change the value:
    global current_wheel_speeds

    # Use the names and speeds send by the msg to generate the dict:
    current_wheel_speeds = dict(zip(msg.name, msg.velocity))


# Get the current pose:
def get_pose():
    # Get the transformlistener:
    listener = tf.TransformListener()

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
    # If there is an exception return None:
    except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        rospy.logerr("Error while trying to get the current pose.\r")
        return None


# Define the function for the motor pwm:
def set_motor_pwm(pca, channel_forward, channel_backward, pwm_value, MAX_PWM):
    # If the provided max PWM value is larger then the limit set it to the limit of 80% PWM:
    limit = 65535*0.8

    # Handle the case that the MAX_PWM is positiv or negativ:
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
        rospy.loginfo(f'Setting motor pwm: {pwm_value} (forward), Channel: {channel_backward}\r')
    else:
        pca.channels[channel_forward].duty_cycle = -pwm_value
        pca.channels[channel_backward].duty_cycle = 0
        rospy.loginfo(f'Setting motor pwm: {-pwm_value} (backward), Channel: {channel_forward}\r')


# Define the function to stop all engines:
def stop_all_motors(pca):
    # This loop can be called if the node is dieing, so the loop isn't in the main loop anymore. For that reason the pins for the engines etc. are defined in this loop too:
    MOTOR_BL = (0, 1)
    MOTOR_FL = (2, 3)
    MOTOR_BR = (4, 5)
    MOTOR_FR = (6, 7)
    MAX_PWM = 65535*0.8

    # Inform the user that the engines are turned off:
    rospy.loginfo("Stopping all motors...\r")

    # Set the PWM values for the engines to 0:
    set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], 0, MAX_PWM)
    set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], 0, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], 0, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], 0, MAX_PWM)


# If the node is shut down, then stop the engines:
def on_shutdown(pca):
    # Call the function to stop the engines:
    stop_all_motors(pca)
    rospy.loginfo("Shutdown of the node; engines have been stopped.\r")


# Define the function for the servo pwm:
def set_servo_pwm(pi, Pin, pwm_value):
    # Work with step size of 5 to smooth the servo movement:
    step_const = 10

    # Check what servo movement should be generated:
    if pwm_value == GRIPPER_CLOSED:
        pwm = GRIPPER_OPEN
        step_size = -step_const
    elif pwm_value == GRIPPER_OPEN:
        pwm = GRIPPER_CLOSED
        step_size = step_const
    else:
        # If there is an other PWM value then write it onto the pin without smoothing it:
        pi.set_servo_pulsewidth(Pin, pwm_value)
        return
    pi.set_servo_pulsewidth(Pin, pwm_value)
    
    # Calculate the amount of steps to smooth the movement:
    steps = int((pwm_value - pwm) / step_size)

    # Process the movement till the last step (skip the last step) and then set the PWM value to the requested PWM value; due to this the PWM value will never leave the defined PWM values due to casting etc.:
    for m in range(steps - 1):
        pwm = int(pwm + step_size)
        pi.set_servo_pulsewidth(Pin, pwm)
        rospy.sleep(0.008)

    # Update the pulswidth for the servo:
    pi.set_servo_pulsewidth(Pin, pwm_value)
    

# Callback for map updates:
def map_callback(msg):
    # Set the latest map variable to global to be able to write onto it:
    global latest_map

    # Generate a deep copy, cause otherwise the variables will never keep the old information of the map / they'll always be updated with this callback too and that's not what is needed in this case:
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


# Search for the pose of the can in the map data:
def calculatePoseCan(map1, map2):
    # Get the resolution and define the margin:
    resolution = map1.info.resolution
    margin = 0.2

    # Calculate the amount of cells to crop the margin from the sides:
    crop_cells = int(margin / resolution)

    # Convert the map data into arrays:
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

    # Calculate the new coords of the origins after the croping:
    new_origin_x1 = old_origin_x1 + margin
    new_origin_y1 = old_origin_y1 + margin
    new_origin_x2 = old_origin_x2 + margin
    new_origin_y2 = old_origin_y2 + margin

    # Calculate the difference of the origins:
    diff_x = new_origin_x2 - new_origin_x1
    diff_y = new_origin_y2 - new_origin_y1

    # Get the max value from the origins. Max is used, cause the origin is located in the 3rd quadrant and due to that the x and y value for the coords are negative 
    # => max will deliver the point that is located closer to the origin of the world coordinatic system:
    min_origin_x = max(new_origin_x1, new_origin_x2)
    min_origin_y = max(new_origin_y1, new_origin_y2)

    # Check if there is an offset of the origins:
    if diff_x != 0 or diff_y != 0:
        # Calculate the distance that has to be croped from the side where the origin is located (left bottom edge):
        offset_x2 = abs(new_origin_x2 - min_origin_x)
        offset_y2 = abs(new_origin_y2 - min_origin_y)

        # Calculate the amount of cells that is equal to the calculated offset:
        offset_cells_x2 = int(round(offset_x2 / resolution))
        offset_cells_y2 = int(round(offset_y2 / resolution))

        # Crop the maps to bring the origins together:
        if offset_cells_y2 == 0:
            map2_cropped = map2_cropped[:, offset_cells_x2:]
        else:
            map2_cropped = map2_cropped[:-offset_cells_y2, offset_cells_x2:]

    # Calculate the min width and height of both maps to define the intersection between both maps:
    final_width = min(map1_cropped.shape[1], map2_cropped.shape[1])
    final_height = min(map1_cropped.shape[0], map2_cropped.shape[0])

    # If the maps have a different shape, correct it:
    if map1_cropped.shape != map2_cropped.shape:
        # Crop the maps to get them to the same shape => due to this both croped maps will have the same intersection.
        # This is needed to calculate the mask and search the can later on:
        map1_cropped = map1_cropped[-final_height:, :final_width]
        map2_cropped = map2_cropped[-final_height:, :final_width]

    # Now the maps should have the same intersection. Calculate the difference in the maps and generate a mask with that information:
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

        # Define the check variable to check if the area is equal to the expected area of the can in the map data:
        check = np.sum(region)*resolution**2*10000

        # Check if the area is within the defined bounds for the expected area of the can:
        if check >= 36 and check <= 64:
            # Append all solutions that deliver an area within the bounds:
            area.append(check)

            # Calculate the CoG of those regions:
            centroid = center_of_mass(region)

            # Calculate the pose in the global coordinate system:
            y_idx, x_idx = centroid
            world_x.append(min_origin_x + x_idx * resolution)
            world_y.append(min_origin_y + (final_height - y_idx) * resolution)

    if world_x and world_y and area:
        return list(zip(world_x, world_y, area))
    else:
        return None


# Function to convert the PID controller value to an PWM value:
def pid_output_to_pwm(corr, v_max=3.0, pwm_max=65535*0.8):
    # Clamping the PID output to the interval of [0, v_max]:
    corr_clamped = max(0, min(corr, v_max))
    
    # Scale the values up to the max PWM value:
    pwm_val = int((corr_clamped / v_max) * pwm_max)
    return pwm_val


# Function to update the PWM vlue for the engines:
def driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR):
    # Add the target variables:
    target_FL = wheel_speeds[0, 0]
    target_FR = wheel_speeds[1, 0]
    target_BL = wheel_speeds[2, 0]
    target_BR = wheel_speeds[3, 0]

    # Set the wheel radius to calculate the angular velocity from the velocity:
    WHEEL_RADIUS = 0.044

    # Current motor speeds in rad/s:
    trueSpeed_FL = current_wheel_speeds.get('FL', 0.0)
    trueSpeed_FR = current_wheel_speeds.get('FR', 0.0)
    trueSpeed_BL = current_wheel_speeds.get('RL', 0.0)
    trueSpeed_BR = current_wheel_speeds.get('RR', 0.0)

    # Define the max speed and max PWM:
    max_speed = 3
    max_pwm=65535*0.8

    """ # Update the setpoint and the constants for the controller:
    pid_FL = PID(0.5, 0.1, 0.02, setpoint=target_FL)
    pid_FR = PID(0.5, 0.1, 0.02, setpoint=target_FR)
    pid_BL = PID(0.5, 0.1, 0.02, setpoint=target_BL)
    pid_BR = PID(0.5, 0.1, 0.02, setpoint=target_BR)

    # Calculate the correction for the engine:
    corr_FL = pid_FL(trueSpeed_FL)
    corr_FR = pid_FR(trueSpeed_FR)
    corr_BL = pid_BL(trueSpeed_BL)
    corr_BR = pid_BR(trueSpeed_BR)

    # Calculte the correct PWM values from the PID controller:
    pwm_fr = pid_output_to_pwm(corr_FR)
    pwm_fl = pid_output_to_pwm(corr_FL)
    pwm_bl = pid_output_to_pwm(corr_BL)
    pwm_br = pid_output_to_pwm(corr_BR)"""

    # Calculate the PMM values:
    pwm_fl = target_FL*max_pwm/max_speed
    pwm_bl = target_BL*max_pwm/max_speed
    pwm_fr = target_FR*max_pwm/max_speed
    pwm_br = target_BR*max_pwm/max_speed

    # Print the true speed and target speed for all wheels:
    rospy.loginfo(f'True Speed FL: {trueSpeed_FL:.3f}, Target: {wheel_speeds[0, 0]:.3f}, PWM: {pwm_fl}\r')
    rospy.loginfo(f'True Speed FR: {trueSpeed_FR:.3f}, Target: {wheel_speeds[1, 0]:.3f},  PWM: {pwm_fr}\r')
    rospy.loginfo(f'True Speed BL: {trueSpeed_BL:.3f}, Target: {wheel_speeds[2, 0]:.3f}, PWM: {pwm_bl}\r')
    rospy.loginfo(f'True Speed BR: {trueSpeed_BR:.3f}, Target: {wheel_speeds[3, 0]:.3f}, PWM: {pwm_br}\r')

    # Check if one of the engines has reached the max PWM value:
    """if pwm_fl > MAX_PWM or pwm_fr > MAX_PWM or pwm_bl > MAX_PWM or pwm_br > MAX_PWM:
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
        set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)"""
    
    
     # Update the PWM values for the engines and the servos:
    set_motor_pwm(pca, MOTOR_FL[0], MOTOR_FL[1], pwm_fl, MAX_PWM)
    set_motor_pwm(pca, MOTOR_FR[0], MOTOR_FR[1], pwm_fr, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BL[0], MOTOR_BL[1], pwm_bl, MAX_PWM)
    set_motor_pwm(pca, MOTOR_BR[0], MOTOR_BR[1], pwm_br, MAX_PWM)

# Setup the function to update the sign of the direction for the speeds of the engines:
def update_motor_signs(wheel_speeds, old_signs, pub, msg):
    # Get the new signs:
    new_signs = [-int(np.sign(wheel_speeds[i, 0])) for i in range(4)]
    
    # If one of the signs has changed, then update the msg:
    if new_signs != old_signs:
        msg.data = new_signs
        pub.publish(msg)
    
    # Return the nem signs to save them as the old signs in the variable in the loop:
    return new_signs


# Define the main function:
def main(pca):
    # Init the node:
    rospy.init_node('dose_sammler')

    # Setup the shut down function for all engines if the node is shuting down:
    rospy.on_shutdown(lambda: on_shutdown(pca))

    # Start the subscribers:
    rospy.Subscriber("/wheel_speeds", JointState, wheel_speed_callback)
    rospy.Subscriber('/keyboard_input', String, key_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    pub = rospy.Publisher('/wheel_directions', Int8MultiArray, queue_size=10)

    # Define all global variables as global:
    global take_map1, take_map2, dx, dy, drot, dGripper, dElevator, poseCanManual, homePose, run, latest_map, current_wheel_speeds

    # Define the channels for the engines and the servos:
    MOTOR_BL = (0, 1)
    MOTOR_FL = (2, 3)
    MOTOR_BR = (4, 5)
    MOTOR_FR = (6, 7)
    PWM_PIN_GRIPPER = 18
    PWM_PIN_ELEVATOR = 10

    # Setup the msg for the signs:
    msg = Int8MultiArray()
    msg.data = [1, 1, 1, 1]
    pub.publish(msg)    

    # Define the max PWM for the engines:
    MAX_PWM = 65535*0.8
    
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

    # Define the index of the port for the USB cam:
    camera_index = '/dev/video0'

    # Setup the pins for the Leuze sensors:
    PINLEUZE1 = 25
    PINLEUZE2 = 9
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PINLEUZE1, GPIO.IN)
    GPIO.setup(PINLEUZE2, GPIO.IN)

    # Define the constants to drive the robot in the autonom mode:
    SPEED_X_AUTONOM = -0.04
    SPEED_Y_AUTONOM = -0.04
    SPEED_ROT_Z_AUTONOM = -0.1

    # Set the bool to run the manual steering of the robot to true:
    run = True

    # Setup pigpio for the PWM pins to control the servos:
    pi = pigpio.pi()
    if not pi.connected:
        exit()

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

    # Setup the old_signs array to be able to calculate the difference of the signs in the first loop:
    old_signs = [int(np.sign(wheel_speeds[i, 0])) for i in range(4)]

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
                rospy.logwarn("You haven't generated a first snap shot without the can on the map.\r")
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

        # Calculate the difference to the values for the movement from the last loop run:
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

        # Update the signs for the odom:
        old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)

        # Update the engine targets:
        driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

        rate.sleep()

    # Set the engine PWM targets to 0:
    stop_all_motors(pca)

    # Check if there are 2 arrays with some map data in it:
    if map1 is not None and map2 is not None:
        rospy.loginfo("Searching for objects in the maps.\r")

        # Calculate the pose of the can:
        pose_of_cans = calculatePoseCan(map1, map2)

        # Define positionCan as an array:
        positionCan = []

        if pose_of_cans:
            # Print all contours in the mask:
            for i, (x, y, a) in enumerate(pose_of_cans):
                rospy.loginfo(f'Pose can: {x:.2f}, {y:.2f}\r')
                positionCan.append((x, y))
                posCans = True
                rospy.loginfo("Can detected.\r")
    else:
        rospy.loginfo("map1 and or map2 are / is empty\r")

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

        # Calculate the yaw of the robot in euler angle:
        quat = [poseOrigin.pose.orientation.w, poseOrigin.pose.orientation.x, poseOrigin.pose.orientation.y, poseOrigin.pose.orientation.z]
        _, _, yaw = quat2euler(quat, axes='sxyz')

        # Correct the orientation from the RoS coordinate system to the robot orientation:
        yaw = yaw - np.pi
        
        rospy.loginfo(f'orientation robot: {yaw}\r')

        # Calculate the difference in the angle pose between the robot and the can and normalize it to be within the -pi to pi interval:
        diff_orient = np.atan2(target_pose_y, target_pose_x) - yaw
        diff_orient = (diff_orient + np.pi) % (2 * np.pi) - np.pi

        # Define an tolerance angle, if the robot cuts this tolerance, so the angle difference is below the tolerance, then the robot can correct the y and x offset to drive towards the can and collect it: 
        angle_tolerance = np.deg2rad(10)

        # Check if the can is within the tolerance:
        while abs(diff_orient) > angle_tolerance:
            # Set the rot target to 0.3 and check in what direction it should turn:
            dx = 0
            dy = 0
            drot = SPEED_ROT_Z_AUTONOM*np.sign(diff_orient)

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Update the signs for the odom:
            old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)

            # Update the PWM targets for the eninges:
            driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the current pose:
            currentPose = get_pose()

            # Calculate the new angle from the new pose:
            _, _, yaw = quat2euler(quat, axes='sxyz')
            quat = [currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z]

            # Correct the orientation from the RoS coordinate system to the robot orientation:
            yaw = yaw - np.pi

            # Calculate the new angle difference from the new pose to the can:
            diff_orient = np.atan2(target_pose_y, target_pose_x) - yaw
            diff_orient = (diff_orient + np.pi) % (2 * np.pi) - np.pi

            rate.sleep()
        
        # Set the engine targets to 0:
        stop_all_motors(pca)

        # Correct the y offset until the robot is within the tolerance:
        while abs(diff_pose_y) > 0.05:
            # Set the y target to drive sideways:
            dx = 0
            dy = SPEED_Y_AUTONOM*np.sign(diff_pose_y)
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Update the signs for the odom:
            old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)

            # Update the PWM targets for the eninges:
            driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the current pose:
            currentPose = get_pose()

            # Calculate the new difference in the pose:
            diff_pose_x = target_pose_x - currentPose.pose.x
            diff_pose_y = target_pose_y - currentPose.pose.y

            rate.sleep()
        
        # Set the engine targets to 0:
        stop_all_motors(pca)

        # Drive towards the can and stop the robot as soon as it's located about 0.5m in front of the can:
        while abs(diff_pose_x) > 0.5:
            # Update the dx and dx target, dx will only trigger if the robot starts to drift:
            dx = SPEED_X_AUTONOM*np.sign(diff_pose_x)
            dy = -diff_pose_y*kp
            drot = 0

            # Calculate the new wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Update the signs for the odom:
            old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)

            # Update the engine targets:
            driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the current pose:
            currentPose = get_pose()

            # Calculate the new difference between the current pose and the target:
            diff_pose_x = target_pose_x - currentPose.pose.x
            diff_pose_y = target_pose_y - currentPose.pose.y

            rate.sleep()
        
        # Set the engine targets to 0:
        stop_all_motors(pca)

        # Check if there is a can based upon the cam data:
        isCan = scc.find_best_can(camera_index)

        # If there is a can; collect it:
        if isCan:
            # Check the Leuze sensors. When they see the can stop the engines:
            while Leuze1 is False and Leuze2 is False:
                # Drive towards the can; no dy correction and no rotation needed here:
                dx = SPEED_X_AUTONOM*np.sign(diff_pose_x)
                dy = 0
                drot = 0

                # Update the wheel speeds:
                wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

                # Update the signs for the odom:
                old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)

                # Update the engine PWM targets:
                driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

                # Read from the Leuze sensors if they can see the can:
                Leuze1 = GPIO.input(PINLEUZE1)
                Leuze2 = GPIO.input(PINLEUZE2)

                rate.sleep()

            # Update the PMW value for the engines and the servos:
            stop_all_motors(pca)

            # Set the PWM values for the servos to grab the can:
            pwm_gripper = GRIPPER_CLOSED
            pwm_elevator = ELEVATOR_TOP

            # Update the PWM for the gripper to grab the can:
            set_servo_pwm(pi, PWM_PIN_GRIPPER, pwm_gripper)

            # Update the elevator PWM value to lift the can:
            set_servo_pwm(pi, PWM_PIN_ELEVATOR, pwm_elevator)

        currentPose = get_pose()

        # Calculate the new angle from the new pose:
        _, _, yaw = quat2euler(quat, axes='sxyz')
        quat = [currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z]

        # Correct the orientation from the RoS coordinate system to the robot orientation:
        yaw = yaw - np.pi

        # Calculate the new angle difference from the new pose to the can:
        diff_orient = yaw + np.pi
        diff_orient = (diff_orient + np.pi) % (2 * np.pi) - np.pi

        # Check if the can is within the tolerance:
        while abs(diff_orient) > angle_tolerance:
            # Set the rot target to 0.3 and check in what direction it should turn:
            dx = 0
            dy = 0
            drot = SPEED_ROT_Z_AUTONOM*np.sign(diff_orient)

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Update the signs for the odom:
            old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)

            # Update the PWM targets for the eninges:
            driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the current pose:
            currentPose = get_pose()

            # Calculate the new angle from the new pose:
            _, _, yaw = quat2euler(quat, axes='sxyz')
            quat = [currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z]

            # Correct the orientation from the RoS coordinate system to the robot orientation:
            yaw = yaw - np.pi

            # Calculate the new angle difference from the new pose to the can:
            diff_orient = yaw + np.pi
            diff_orient = (diff_orient + np.pi) % (2 * np.pi) - np.pi

            rate.sleep()
        
        # Set the engine targets to 0:
        stop_all_motors(pca)

        # Get the current pose:
        pose = get_pose()

        # Calculate the offset from the current pose and the home pose:
        pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
        pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y

        # While this offset is to big update the PWM values to get closer to the home pose:
        while abs(pose_offset_y) > 0.05:
            # Calculate the new speed targets:
            dx = 0
            dy = SPEED_Y_AUTONOM*np.sign(pose_offset_y)
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Update the signs for the odom:
            old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)
            
            # Update the engine targets:
            driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the new pose:
            pose = get_pose()

            # Calculate the new offset:
            pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
            pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y

            rate.sleep()
        
        # Stop all engines:
        stop_all_motors(pca)
        
        # While this offset is to big update the PWM values to get closer to the home pose:
        while abs(pose_offset_x) > 0.05:
            # Calculate the new speed targets:
            dx = SPEED_X_AUTONOM*np.sign(pose_offset_x)
            dy = -pose_offset_y*kp
            drot = 0

            # Update the wheel speeds:
            wheel_speeds = mecanum_inv_kinematics(dx, dy, drot)

            # Update the signs for the odom:
            old_signs = update_motor_signs(wheel_speeds, old_signs, pub, msg)
            
            # Update the engine targets:
            driveEngines(wheel_speeds, MAX_PWM, pca, MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR)

            # Get the new pose:
            pose = get_pose()

            # Calculate the new offset:
            pose_offset_x = poseOrigin.pose.position.x - pose.pose.position.x
            pose_offset_y = poseOrigin.pose.position.y - pose.pose.position.y

            rate.sleep()
        
        # Stop all engines:
        stop_all_motors(pca)

        # Set the elevator PWM value to the bottom state:
        pwm_elevator = ELEVATOR_BOTTOM

        # Update the PWM value for the elevator to lower the can:
        set_servo_pwm(pi, PWM_PIN_ELEVATOR, pwm_elevator)

        # Set the gripper PWM value to the open state:
        pwm_gripper = GRIPPER_OPEN

        # Update the PWM value for the gripper to open the gripper:
        set_servo_pwm(pi, PWM_PIN_GRIPPER, pwm_gripper)

    # End of the program:
    rospy.loginfo("Program has finished.\r")
    rospy.signal_shutdown("User interrupt ends the program.\r")
    sys.exit(0)

if __name__ == '__main__':
    # Initalise the I2C:
    i2c = busio.I2C(SCL, SDA)

    # Initalise the PWM board to control the engines, 1000 Hz is the max PWM value for that board:
    pca = PCA9685(i2c)
    pca.frequency = 1000

    # Try to execute the main loop:
    try:
        main(pca)
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'ROS has exited with the exception: {e}\r')
    except Exception as e:
        rospy.logerr(f'Unexpected error: {e}\r')
    # Shut down all engines and clean the GPIOs at the end:
    finally:
        stop_all_motors(pca)
        GPIO.cleanup()
        rospy.loginfo("Final cleanup done. Exiting.\r")
        sys.exit(0)
