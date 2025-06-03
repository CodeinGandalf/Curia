#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy
import RPi.GPIO as GPIO
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

# Define the pins for the encoder:
ENCODER_PINS = {
    "FL_A": 22, "FL_B": 27,
    "FR_A": 24, "FR_B": 23,
    "RL_A": 19, "RL_B": 13,
    "RR_A": 11, "RR_B": 0
}

# Tick counter:
import threading
ticks = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
ticks_lock = threading.Lock()

# Store direction for each wheel (default to 1)
encoder_dir = {"FL": 1, "FR": 1, "RL": 1, "RR": 1}
encoder_dir_lock = threading.Lock()

# Callback for direction topic
def dir_callback(msg):
    # msg should be a dictionary-like message, e.g. {'FL': 1, 'FR': 1, 'RL': -1, 'RR': -1}
    # For example, use std_msgs/Int32MultiArray or a custom message
    with encoder_dir_lock:
        encoder_dir['FL'] = msg.data[0]
        encoder_dir['FR'] = msg.data[1]
        encoder_dir['RL'] = msg.data[2]
        encoder_dir['RR'] = msg.data[3]

# Unified encoder callback
def encoder_callback(channel, wheel_name):
    # Only increment/decrement on rising edge of encoder A
    with encoder_dir_lock:
        dir = encoder_dir[wheel_name]
    with ticks_lock:
        ticks[wheel_name] += dir

# -------------- UPDATED SETUP FUNCTION --------------


def setup_encoders():
    GPIO.setmode(GPIO.BCM)
    # Define wheel info: (A pin, direction)
    wheel_info = {
        "FL": (ENCODER_PINS["FL_A"],  1),
        "FR": (ENCODER_PINS["FR_A"],  1),
        "RL": (ENCODER_PINS["RL_A"], -1),
        "RR": (ENCODER_PINS["RR_A"], -1),
    }
    for wheel, (a_pin, dir) in wheel_info.items():
        GPIO.setup(a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Use lambda to pass wheel_name and dir to the callback
        GPIO.add_event_detect(
            a_pin, GPIO.RISING,
            callback=lambda channel, w=wheel, d=dir: encoder_callback(channel, w, d)
        )


# Define the function to calculate the odometry:
def calculate_odometry(ticks_delta, dt):
    TICKS_PER_REV = 700 #2797
    WHEEL_RADIUS = 0.044
    L = 0.244
    W = 0.132

    w = {}

    wheel_circumference = 2 * math.pi * WHEEL_RADIUS
    for name in ticks_delta:
        revs = ticks_delta[name] / TICKS_PER_REV
        w[name] = (wheel_circumference * revs) / dt

    # Inverse kinematics:
    vx = (w["FL"] + w["FR"] + w["RL"] + w["RR"]) / 4
    vy = (-w["FL"] + w["FR"] + w["RL"] - w["RR"]) / 4
    wz = (-w["FL"] - w["FR"] + w["RL"] + w["RR"]) / (4 * (L + W))

    return vx, vy, wz, w


# Define the main function:
def main():
    # Init the node:
    rospy.init_node("encoder_odometry")
    # Unterhalb von odom_pub diese Zeile einfuegen:
    joint_pub = rospy.Publisher("/wheel_speeds", JointState, queue_size=10)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to direction topic (example: Int32MultiArray)
    rospy.Subscriber("/wheel_direction", Int8MultiArray, dir_callback)

    # Setup the encoders:
    setup_encoders()

    # Define the rate and further needed parameters:
    rate = rospy.Rate(100)  # 100 Hz
    x = y = th = 0.0
    last_ticks = ticks.copy()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # Calculate the passed time:
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
    
        if dt == 0:
            rate.sleep()
            continue

        # Calculate the amount of ticks:
        ticks_delta = {k: ticks[k] - last_ticks[k] for k in ticks}
        last_ticks = ticks.copy()
        
        # Calculate the odometry:
        vx, vy, vth, w = calculate_odometry(ticks_delta, dt)

        # Calculate dx, dy and drot:
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # Calculate the quaternion from the parameters:
        odom_quat = quaternion_from_euler(0, 0, th)

        # Define the transformation:
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = -x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]
        tf_broadcaster.sendTransform(t)

        # Get the odometry:
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # JointState Nachricht mit Ticks & Radgeschwindigkeit
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['FL', 'FR', 'RL', 'RR']
        joint_state.position = [ticks['FL'], ticks['FR'], ticks['RL'], ticks['RR']]
        joint_state.velocity = [w['FL'], w['FR'], w['RL'], w['RR']]
        joint_state.effort = [0.0, 0.0, 0.0, 0.0]

        joint_pub.publish(joint_state)

        # Publish the odom:
        odom_pub.publish(odom)
        last_time = now
        rate.sleep()

# Call the main function if it's executed as the main file:
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
