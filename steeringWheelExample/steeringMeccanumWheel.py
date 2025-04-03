import numpy as np
from simple_pid import PID
import time
import matplotlib.pyplot as plt


def mecanum_inv_kinematics(vx, vy, omega, wheelRadius, wheelBase, trackWidth):
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


# Test values for the function:
dx = 0.22
dy = 0.11
rot = 0.0
r = 0.03
L = 0.2
W = 0.15
iEngine = 1
incEngine = 50
dt_FL = 0.01
dt_FR = 0.01
dt_BL = 0.01
dt_BR = 0.01
deltaIncRad = 2 * np.pi / incEngine

# Calculate the target wheel speed
wheel_speeds = mecanum_inv_kinematics(dx, dy, rot, r, L, W)

dt_FL_soll = deltaIncRad * iEngine / wheel_speeds[0, 0]
dt_FR_soll = deltaIncRad * iEngine / wheel_speeds[1, 0]
dt_BL_soll = deltaIncRad * iEngine / wheel_speeds[2, 0]
dt_BR_soll = deltaIncRad * iEngine / wheel_speeds[3, 0]

# Adjusted PID controllers for smoother response
pid_FL = PID(0.5, 0.1, 0.02, setpoint=dt_FL_soll)
pid_FR = PID(0.5, 0.1, 0.02, setpoint=dt_FR_soll)
pid_BL = PID(0.5, 0.1, 0.02, setpoint=dt_BL_soll)
pid_BR = PID(0.5, 0.1, 0.02, setpoint=dt_BR_soll)

trueSpeed_FL = deltaIncRad / dt_FL * iEngine
trueSpeed_FR = deltaIncRad / dt_FR * iEngine
trueSpeed_BL = deltaIncRad / dt_BL * iEngine
trueSpeed_BR = deltaIncRad / dt_BR * iEngine
timestart = time.time()

run = True
distFL = np.array([])
distFR = np.array([])
distBL = np.array([])
distBR = np.array([])
realdistFL = np.array([])
realdistFR = np.array([])
realdistBL = np.array([])
realdistBR = np.array([])
passedTime = np.array([])

# Simulation loop
while run == True:
    # Current motor speeds
    trueSpeed_FL = deltaIncRad / dt_FL * iEngine
    trueSpeed_FR = deltaIncRad / dt_FR * iEngine
    trueSpeed_BL = deltaIncRad / dt_BL * iEngine
    trueSpeed_BR = deltaIncRad / dt_BR * iEngine

    # Compute new dt using PID controllers
    new_dt_FL = pid_FL(dt_FL)
    new_dt_FR = pid_FR(dt_FR)
    new_dt_BL = pid_BL(dt_BL)
    new_dt_BR = pid_BR(dt_BR)

    # Slow down rate of dt adjustment to prevent overshoot
    dt_FL += new_dt_FL
    dt_FR += new_dt_FR
    dt_BL += new_dt_BL
    dt_BR += new_dt_BR

    # Print the true speed and target speed for all wheels
    print(
        f'True Speed FL: {trueSpeed_FL:.3f}, Target: {wheel_speeds[0, 0]:.3f}')
    print(
        f'True Speed FR: {trueSpeed_FR:.3f}, Target: {wheel_speeds[1, 0]:.3f}')
    print(
        f'True Speed BL: {trueSpeed_BL:.3f}, Target: {wheel_speeds[2, 0]:.3f}')
    print(
        f'True Speed BR: {trueSpeed_BR:.3f}, Target: {wheel_speeds[3, 0]:.3f}')
    print()

    """if all(abs(trueSpeed - wheel_speeds[i, 0]) < 0.002 for i, trueSpeed
           in enumerate([trueSpeed_FL, trueSpeed_FR, trueSpeed_BL,
                         trueSpeed_BR])):
        dx += 0.01
        dy += 0.01
        rot += 0.01

        # Calculate the target wheel speed
        wheel_speeds = mecanum_inv_kinematics(dx, dy, rot, r, L, W)

        dt_FL_soll = deltaIncRad * iEngine / wheel_speeds[0, 0]
        dt_FR_soll = deltaIncRad * iEngine / wheel_speeds[1, 0]
        dt_BL_soll = deltaIncRad * iEngine / wheel_speeds[2, 0]
        dt_BR_soll = deltaIncRad * iEngine / wheel_speeds[3, 0]

        # Adjusted PID controllers for smoother response
        pid_FL = PID(0.5, 0.1, 0.02, setpoint=dt_FL_soll)
        pid_FR = PID(0.5, 0.1, 0.02, setpoint=dt_FR_soll)
        pid_BL = PID(0.5, 0.1, 0.02, setpoint=dt_BL_soll)
        pid_BR = PID(0.5, 0.1, 0.02, setpoint=dt_BR_soll)"""

    current_time = time.time()
    delta = current_time - timestart

    if (current_time - timestart) > 20:
        run = False

    distFL = np.append(distFL, trueSpeed_FL*delta)
    distFR = np.append(distFR, trueSpeed_FR*delta)
    distBL = np.append(distBL, trueSpeed_BL*delta)
    distBR = np.append(distBR, trueSpeed_BR*delta)
    realdistFL = np.append(realdistFL, wheel_speeds[0, 0]*delta)
    realdistFR = np.append(realdistFR, wheel_speeds[1, 0]*delta)
    realdistBL = np.append(realdistBL, wheel_speeds[2, 0]*delta)
    realdistBR = np.append(realdistBR, wheel_speeds[3, 0]*delta)
    passedTime = np.append(passedTime, delta)

    # Delay for simulation
    time.sleep(0.03)

# Plotting the distances for each wheel over time
plt.figure(figsize=(10, 6))
plt.plot(passedTime, distFL, label="Front Left Wheel")
plt.plot(passedTime, distFR, label="Front Right Wheel")
plt.plot(passedTime, distBL, label="Back Left Wheel")
plt.plot(passedTime, distBR, label="Back Right Wheel")
plt.plot(passedTime, realdistFL, label="real Front Left Wheel")
plt.plot(passedTime, realdistFR, label="real Front Right Wheel")
plt.plot(passedTime, realdistBL, label="real Back Left Wheel")
plt.plot(passedTime, realdistBR, label="real Back Right Wheel")
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.title('Distance Traveled by Each Wheel Over Time')
plt.legend()
plt.grid(True)
plt.show()
