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

# Define the pins for the encoder:
ENCODER_PINS = {
    "FL_A": 19, "FL_B": 13,
    "FR_A": 11, "FR_B": 0,
    "RL_A": 22, "RL_B": 27,
    "RR_A": 24, "RR_B": 23
}

# Tick counter:
import threading
ticks = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
ticks_lock = threading.Lock()


def encoder_callback_FL(pin_a, pin_b, last_states_FL, wheel_name="FL"):
    def _callback(channel):
        a, b = GPIO.input(pin_a), GPIO.input(pin_b)
        current_state = (a << 1) | b
        
        transition = (last_states_FL[0] << 2) | current_state

        # Alle gueltigen Uebergaenge mit Richtung
        with ticks_lock:
            if transition in [0b0001, 0b0111, 0b1110, 0b1000]:
                ticks[wheel_name] += 1
            elif transition in [0b0010, 0b0100, 0b1101, 0b1011]:
                ticks[wheel_name] -= 1

        last_states_FL[0] = current_state
    return _callback


def encoder_callback_FR(pin_a, pin_b, last_states_FR, wheel_name="FR"):
    def _callback(channel):
        a, b = GPIO.input(pin_a), GPIO.input(pin_b)
        current_state = (a << 1) | b

        transition = (last_states_FR[0] << 2) | current_state

        # Alle gueltigen Uebergaenge mit Richtung
        with ticks_lock:
            if transition in [0b0001, 0b0111, 0b1110, 0b1000]:
                ticks[wheel_name] += 1
            elif transition in [0b0010, 0b0100, 0b1101, 0b1011]:
                ticks[wheel_name] -= 1

        last_states_FR[0] = current_state
    return _callback

    
def encoder_callback_RL(pin_a, pin_b, last_states_RL, wheel_name="RL"):
    def _callback(channel):
        a, b = GPIO.input(pin_a), GPIO.input(pin_b)
        current_state = (a << 1) | b

        transition = (last_states_RL[0] << 2) | current_state

        # Alle gueltigen Uebergaenge mit Richtung
        with ticks_lock:
            if transition in [0b0001, 0b0111, 0b1110, 0b1000]:
                ticks[wheel_name] += 1
            elif transition in [0b0010, 0b0100, 0b1101, 0b1011]:
                ticks[wheel_name] -= 1

        last_states_RL[0] = current_state
    return _callback


def encoder_callback_RR(pin_a, pin_b, last_states_RR, wheel_name="RR"):
    def _callback(channel):
        a, b = GPIO.input(pin_a), GPIO.input(pin_b)
        current_state = (a << 1) | b

        transition = (last_states_RR[0] << 2) | current_state

        # Alle gueltigen Uebergaenge mit Richtung
        with ticks_lock:
            if transition in [0b0001, 0b0111, 0b1110, 0b1000]:
                ticks[wheel_name] += 1
            elif transition in [0b0010, 0b0100, 0b1101, 0b1011]:
                ticks[wheel_name] -= 1

        last_states_RR[0] = current_state
    return _callback


# Define the function to setup the encoder:
def setup_encoders():
    GPIO.setmode(GPIO.BCM)
    for wheel in ["FL", "FR", "RL", "RR"]:
        a_pin = ENCODER_PINS[f"{wheel}_A"]
        b_pin = ENCODER_PINS[f"{wheel}_B"]
        GPIO.setup(a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    state_FL = [00]
    state_FR = [00]
    state_RL = [00]
    state_RR = [00]
    cb_FL = encoder_callback_FL(ENCODER_PINS["FL_A"], ENCODER_PINS["FL_B"], state_FL)
    cb_FR = encoder_callback_FR(ENCODER_PINS["FR_A"], ENCODER_PINS["FR_B"], state_FR)
    cb_RL = encoder_callback_RL(ENCODER_PINS["RL_A"], ENCODER_PINS["RL_B"], state_RL)
    cb_RR = encoder_callback_RR(ENCODER_PINS["RR_A"], ENCODER_PINS["RR_B"], state_RR)

    GPIO.add_event_detect(ENCODER_PINS["FL_A"], GPIO.BOTH, callback=cb_FL)
    GPIO.add_event_detect(ENCODER_PINS["FL_B"], GPIO.BOTH, callback=cb_FL)

    GPIO.add_event_detect(ENCODER_PINS["FR_A"], GPIO.BOTH, callback=cb_FR)
    GPIO.add_event_detect(ENCODER_PINS["FR_B"], GPIO.BOTH, callback=cb_FR)

    GPIO.add_event_detect(ENCODER_PINS["RL_A"], GPIO.BOTH, callback=cb_RL)
    GPIO.add_event_detect(ENCODER_PINS["RL_B"], GPIO.BOTH, callback=cb_RL)

    GPIO.add_event_detect(ENCODER_PINS["RR_A"], GPIO.BOTH, callback=cb_RR)
    GPIO.add_event_detect(ENCODER_PINS["RR_B"], GPIO.BOTH, callback=cb_RR)


# Define the function to calculate the odometry:
def calculate_odometry(ticks_delta, dt):
    TICKS_PER_REV = 2797
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
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
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
