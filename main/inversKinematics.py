import numpy as np


def mecanum_inv_kinematics(vx, vy, omega, wheelRadius=0.03, wheelBase=0.2, trackWidth=0.15):
    L = wheelBase / 2
    W = trackWidth / 2

    # Matrix for the inverse kinematics
    J_inv = 1/wheelRadius*np.matrix([
        [1, -1, -(L + W)],
        [1, 1, (L + W)],
        [1, 1, -(L + W)],
        [1, -1, (L + W)]
    ])

    velocityVector = np.matrix([vx, vy, omega]).T
    wheelSpeeds = J_inv * velocityVector

    return wheelSpeeds.reshape(-1, 1)  # Convert matrix to a vertical array
