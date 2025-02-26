import time
import pigpio
import RPi.GPIO as GPIO
from Adafruit_BNO055 import BNO055

# BNO055 Setup
BNO055_ADDRESS = 0x28
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=6)

# Servo Setup
servo1, servo2 = 4, 27  
pwm = pigpio.pi()
pwm.set_mode(servo1, pigpio.OUTPUT)
pwm.set_mode(servo2, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo1, 50)
pwm.set_PWM_frequency(servo2, 50)

minVal1, maxVal1, centerVal1 = 944, 2278, 1833
minVal2, maxVal2 = int(1475 - 6 / 0.009), int(1475 + 6 / 0.009)
centerVal2 = (maxVal2 + minVal2) / 2

pwm.set_servo_pulsewidth(servo1, centerVal1)
pwm.set_servo_pulsewidth(servo2, centerVal2)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)

with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
    timestamp = time.strftime("      %a %d-%m-%Y @ %H:%M:%S\n")
    f.write("-------------------------\n" + timestamp)
    g.write("-------------------------\n" + timestamp)

if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055!')

# Calibration
for _ in range(85):
    heading, roll, pitch = bno.read_euler()
    sys, gyro, accel, mag = bno.get_calibration_status()
    print(f'Heading={heading:.2f} Roll={roll:.2f} Pitch={pitch:.2f} '
          f'Sys_cal={sys} Gyro_cal={gyro} Accel_cal={accel} Mag_cal={mag}')
    time.sleep(1)

with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
    log_data = (f'Heading={heading:.2f} Roll={roll:.2f} Pitch={pitch:.2f} '
                f'Sys_cal={sys} Gyro_cal={gyro} Accel_cal={accel} Mag_cal={mag}\n')
    f.write("Latest Calibration Data\n" + log_data)
    g.write("Latest Calibration Data\n" + log_data)

while input("Enter 2 to continue: ") != '2':
    pass

desiredPitch = 180 - pitch if pitch > 0 else -180 - pitch
desiredRoll = roll
print(f"Desired pitch = {desiredPitch:.2f} deg, Desired roll = {desiredRoll:.2f} deg")

while input("Enter 2 to continue: ") != '2':
    pass

maxAZ, gx, gy, gz = -14, 0, 0, 0

while True:
    time.sleep(0.02)
    ax, ay, az = bno.read_accelerometer()
    with open("IMU.txt", "a") as f:
        f.write(f'{heading}, {roll}, {pitch}, {ax}, {ay}, {az}, {gx}, {gy}, {gz}, {time.time()*1000:.0f}, {centerVal1}, {centerVal2}\n')
    
    if az < maxAZ:
        time.sleep(0.03)
        ax, ay, az = bno.read_accelerometer()
        if az < maxAZ:
            time.sleep(0.03)
            ax, ay, az = bno.read_accelerometer()
            if az < maxAZ:
                with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
                    f.write("Launch Detected\n")
                    g.write("Launch Detected\n")
                break

time_start = time.time() * 1000
Kp, Kd, Ki = 45 * 3.14 / 180, 10 * 3.14 / 180, 0.1 * 3.14 / 180
roll_integral, roll_old, pitch_old, pitch_integral = 0, 0, 0, 0
servoYval = centerVal2
ejectionfirecheck = True

while (time.time() * 1000 - time_start) < 55000:
    heading, roll, pitch = bno.read_euler()
    gx, gy, gz = bno.read_gyroscope()
    ax, ay, az = bno.read_accelerometer()
    
    if (time.time() * 1000 - time_start) > 4500 and ejectionfirecheck:
        ejectionfirecheck = False
        GPIO.output(20, GPIO.HIGH)
        with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
            f.write(f"Fired Ejection Charge - {time.time()*1000:.0f}\n")
            g.write(f"Fired Ejection Charge - {time.time()*1000:.0f}\n")

    roll_derivative = gy * 57.32
    roll_integral += (roll - desiredRoll) * 0.001
    rollSetpoint = -Kp * (roll - desiredRoll) - Kd * roll_derivative - Ki * roll_integral
    servoYval = centerVal2 - rollSetpoint / 0.009
    servoYval = max(minVal2, min(maxVal2, servoYval))

    pitchVal = 180 - pitch if pitch > 0 else -180 - pitch
    pitch_derivative = gx * 57.32
    pitch_integral += (pitchVal - desiredPitch) * 0.001
    pitchSetpoint = -Kp * (pitchVal - desiredPitch) - Kd * pitch_derivative - Ki * pitch_integral
    servoXval = centerVal1 - pitchSetpoint / 0.009
    servoXval = max(minVal1, min(maxVal1, servoXval))

    if ejectionfirecheck:
        pwm.set_servo_pulsewidth(servo2, servoYval)
        pwm.set_servo_pulsewidth(servo1, servoXval)

    roll_old, pitch_old = roll, pitchVal

    with open("IMU.txt", "a") as f:
        f.write(f'{heading}, {roll}, {pitchVal}, {ax}, {ay}, {az}, {gx}, {gy}, {gz}, {time.time()*1000:.0f}, {rollSetpoint}, {pitchSetpoint}\n')

pwm.set_servo_pulsewidth(servo1, centerVal1)
pwm.set_servo_pulsewidth(servo2, centerVal2)

with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
    f.write("-------------------------\n")
    g.write("-------------------------\n")
