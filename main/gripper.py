# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""


def gripperClose():
    closedPWM = 100
    gripperPWM = 1000
    PWM_STEP = 15

    while gripperPWM > closedPWM:
        gripperPWM -= PWM_STEP
        print(gripperPWM)

    """
    Update the PWM for the Gripper Servo.
    """


def gripperOpen():
    openPWM = 1000
    gripperPWM = 100
    PWM_STEP = 15

    while gripperPWM > openPWM:
        gripperPWM += PWM_STEP
        print(gripperPWM)

    """
    Update the PWM for the Gripper Servo.
    """


def servoUpper():
    maxLiftPWM = 1000
    liftPWM = 100
    PWM_STEP = 15

    while liftPWM < maxLiftPWM:
        liftPWM += PWM_STEP
        print(liftPWM)

    """
    Update the PWM for the lift Servo.
    """


def servoLower():
    minLiftPWM = 100
    liftPWM = 1000
    PWM_STEP = 15

    while liftPWM > minLiftPWM:
        liftPWM += PWM_STEP
        print(liftPWM)

    """
    Update the PWM for the lift Servo.
    """
