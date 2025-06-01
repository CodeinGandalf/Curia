#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import termios
import tty


# Define the function to read the keys:
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


# Setup the function to read the keyboard
def main():
    rospy.init_node('keyboard_control')
    pub = rospy.Publisher('/keyboard_input', String, queue_size=10)
    rate = rospy.Rate(10)

    rospy.loginfo("Keyboard keys: W/S/A/D = drive, E/Q = turn, P = stop, R/F = elevator, T/G = gripper, 0 = break, H = home pose")

    run_node = True

    while not rospy.is_shutdown() and run_node:
        key = get_key()

        if key == '0':  # ESC
            rospy.loginfo("Node shut down.\n")
            run_node = False

        # Publish the detected key:
        pub.publish(key)
        rate.sleep()


# Call the main funcion if the file is executed as main file:
if __name__ == '__main__':
    main()
