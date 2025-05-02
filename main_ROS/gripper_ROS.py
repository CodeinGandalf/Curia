# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import rospy


def gripperClose():
    rospy.loginfo("Hello from gripper close")
    closedPWM = 100
    gripperPWM = 1000
    PWM_STEP = 15

    while gripperPWM > closedPWM:
        gripperPWM -= PWM_STEP
        rospy.loginfo(gripperPWM)

    """
    Update the PWM for the Gripper Servo.
    """


def gripperOpen():
    rospy.loginfo("Hello from gripper open")
    openPWM = 1000
    gripperPWM = 100
    PWM_STEP = 15

    while gripperPWM > openPWM:
        gripperPWM += PWM_STEP
        rospy.loginfo(gripperPWM)

    """
    Update the PWM for the Gripper Servo.
    """


def servoUpper():
    rospy.loginfo("Hello from servo upper")
    maxLiftPWM = 1000
    liftPWM = 100
    PWM_STEP = 15

    while liftPWM < maxLiftPWM:
        liftPWM += PWM_STEP
        rospy.loginfo(liftPWM)

    """
    Update the PWM for the lift Servo.
    """


def servoLower():
    rospy.loginfo("Hello from servo lower")
    minLiftPWM = 100
    liftPWM = 1000
    PWM_STEP = 15

    while liftPWM > minLiftPWM:
        liftPWM += PWM_STEP
        rospy.loginfo(liftPWM)

    """
    Update the PWM for the lift Servo.
    """
