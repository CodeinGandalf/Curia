from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
import rospy
import time

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60

print("PCA9685 gefunden und auf 60 Hz eingestellt")

# PWM-Schrittweite: 5 % von 4096
max_pwm = 65535  # 4096 (12-bit)
step = int(max_pwm * 0.05)  # 5% Schritt
value = 0

# Alle 5 Sekunden den PWM-Wert erhöhen
while value <= max_pwm:
    pca.channels[0].duty_cycle = value
    print(f"PWM-Wert auf Kanal 0 gesetzt: {value} ({round(value / max_pwm * 100)}%)")
    value += step
    time.sleep(5)

# Abschalten nach Erreichen 100 %
print("Maximalwert erreicht. PWM auf 0 zurücksetzen.")
pca.channels[0].duty_cycle = 0

