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
last_time = timestart

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

    current_time = time.time()
    delta = current_time - timestart

    if current_time - 2 >= last_time:
        last_time = current_time
        """
        dx += 0.01
        dy += 0.01
        rot += 0.00
        """

        # Calculate the target wheel speed
        wheel_speeds = mecanum_inv_kinematics(dx, dy, rot, r, L, W)

        dt_FL_soll = deltaIncRad * iEngine / wheel_speeds[0, 0]
        dt_FR_soll = deltaIncRad * iEngine / wheel_speeds[1, 0]
        dt_BL_soll = deltaIncRad * iEngine / wheel_speeds[2, 0]
        dt_BR_soll = deltaIncRad * iEngine / wheel_speeds[3, 0]

        # Adjusted PID controllers for smoother response
        pid_FL.setpoint = dt_FL_soll
        pid_FR.setpoint = dt_FR_soll
        pid_BL.setpoint = dt_BL_soll
        pid_BR.setpoint = dt_BR_soll

    if (current_time - timestart) > 50:
        run = False

    distFL = np.append(distFL, trueSpeed_FL)
    distFR = np.append(distFR, trueSpeed_FR)
    distBL = np.append(distBL, trueSpeed_BL)
    distBR = np.append(distBR, trueSpeed_BR)
    realdistFL = np.append(realdistFL, wheel_speeds[0, 0])
    realdistFR = np.append(realdistFR, wheel_speeds[1, 0])
    realdistBL = np.append(realdistBL, wheel_speeds[2, 0])
    realdistBR = np.append(realdistBR, wheel_speeds[3, 0])
    passedTime = np.append(passedTime, delta)

    # Delay for simulation
    time.sleep(0.03)

# Subplot erstellen
fig, axs = plt.subplots(2, 2, figsize=(14, 10))

# Plot für Front Left Wheel und Geschwindigkeit
axs[0, 0].plot(passedTime, distFL, label="Front Left Wheel")
axs[0, 0].plot(passedTime, realdistFL, label="Real Front Left", linestyle='--')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Distance (m) / Speed (m/s)')
axs[0, 0].set_title('Front Left Wheel')
axs[0, 0].legend()
axs[0, 0].grid(True)
# axs[0, 0].set_xscale('log')  # Logarithmische x-Achse
# axs[0, 0].set_yscale('log')  # Logarithmische y-Achse

# Plot für Front Right Wheel und Geschwindigkeit
axs[0, 1].plot(passedTime, distFR, label="Front Right Wheel")
axs[0, 1].plot(passedTime, realdistFR,
               label="Real Front Right", linestyle='--')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Distance (m) / Speed (m/s)')
axs[0, 1].set_title('Front Right Wheel')
axs[0, 1].legend()
axs[0, 1].grid(True)
# axs[0, 1].set_xscale('log')  # Logarithmische x-Achse
# axs[0, 1].set_yscale('log')  # Logarithmische y-Achse

# Plot für Back Left Wheel und Geschwindigkeit
axs[1, 0].plot(passedTime, distBL, label="Back Left Wheel")
axs[1, 0].plot(passedTime, realdistBL, label="Real Back Left", linestyle='--')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Distance (m) / Speed (m/s)')
axs[1, 0].set_title('Back Left Wheel')
axs[1, 0].legend()
axs[1, 0].grid(True)
# axs[1, 0].set_xscale('log')  # Logarithmische x-Achse
# axs[1, 0].set_yscale('log')  # Logarithmische y-Achse

# Plot für Back Right Wheel und Geschwindigkeit
axs[1, 1].plot(passedTime, distBR, label="Back Right Wheel")
axs[1, 1].plot(passedTime, realdistBR, label="Real Back Right", linestyle='--')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Distance (m) / Speed (m/s)')
axs[1, 1].set_title('Back Right Wheel')
axs[1, 1].legend()
axs[1, 1].grid(True)
# axs[1, 1].set_xscale('log')  # Logarithmische x-Achse
# axs[1, 1].set_yscale('log')  # Logarithmische y-Achse

plt.tight_layout()
plt.show()
