# BNO055 default I2C address
BNO055_ADDRESS = 0x28

# BNO055 Registers
BNO055_CHIP_ID = 0x00
BNO055_OPR_MODE = 0x3D
BNO055_ACCEL_DATA = 0x08
OPERATION_MODE_NDOF = 0x0C

import logging
import sys
import math
import time
import pigpio
import RPi.GPIO as GPIO

from Adafruit_BNO055 import BNO055

# Initialize servos
servo1 = 4  # X-servo
servo2 = 27  # Y-servo
pwm = pigpio.pi()
pwm.set_mode(servo1, pigpio.OUTPUT)
pwm.set_mode(servo2, pigpio.OUTPUT)

pwm.set_PWM_frequency(servo1, 50)
pwm.set_PWM_frequency(servo2, 50)

# Centering servo positions
pwm.set_servo_pulsewidth(servo1, 1833)
pwm.set_servo_pulsewidth(servo2, 1833)

# Servo bounds
minVal1 = 944
maxVal1 = 2278
centerVal1 = 1833
pwm.set_servo_pulsewidth(servo1, centerVal1)

minVal2 = int(1475 - 6 / 0.009)  # For maxDegreesY = 6
maxVal2 = int(1475 + 6 / 0.009)
centerVal2 = (maxVal2 + minVal2) / 2
pwm.set_servo_pulsewidth(servo2, centerVal2)

# Initialize log files
with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
    f.write("-------------------------")
    f.write(time.strftime("      %a %d-%m-%Y @ %H:%M:%S\n"))
    g.write("-------")
    g.write(time.strftime("      %a %d-%m-%Y @ %H:%M:%S\n"))

current_milli_time = lambda: int(round(time.time() * 1000))
print(f"current_milli_time= {current_milli_time()}")

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=6)

# Change accelerometer range to 16G
time.sleep(0.1)
savePageID = bno._read_byte(0x07)
time.sleep(0.1)
bno._write_byte(0x07, 0x01)
time.sleep(0.1)
print(bno._read_byte(0x07))
time.sleep(0.1)
print(bno._read_byte(0x08))
time.sleep(0.1)
bno._write_byte(0x08, 0xFF)
time.sleep(0.1)
print(bno._read_byte(0x08))
time.sleep(0.1)
print("got")
bno._write_byte(0x07, savePageID & 0xFF)
print("here")

# Debug logging
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize BNO055
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# System status and self-test
status, self_test, error = bno.get_system_status()
print(f"System status: {status}")
print(f"Self test result (0x0F is normal): 0x{self_test:02X}")
if status == 0x01:
    print(f"System error: {error}\nSee datasheet section 4.3.59 for the meaning.")

# Print diagnostic data
sw, bl, accel, mag, gyro = bno.get_revision()
print(f"Software version: {sw}")
print(f"Accelerometer ID: 0x{accel:02X}")
print(f"Magnetometer ID: 0x{mag:02X}")
print(f"Gyroscope ID: 0x{gyro:02X}\n")

print('Reading BNO055 data, press Ctrl-C to quit...')

print('starting 60 seconds with no logging')
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)    # if using same script as IMU, this must be BCM. Can be Board for Pin number otherwise
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)  # pin 38=GPIO20, MOSFET gate for ejection charge
print('Ejection Charge Set Low')

###  calibrate loop #########
for n in range(85):
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
            heading, roll, pitch, sys, gyro, accel, mag))
    time.sleep(1)

with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
    f.write("Latest Calibration Data\n")
    f.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
            heading, roll, pitch, sys, gyro, accel, mag))
    g.write("Latest Calibration Data\n")
    g.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
            heading, roll, pitch, sys, gyro, accel, mag))

print("We should be calibrated on the IMU now, if not restart")
time.sleep(1)
print("You should be putting the rocket on the launchpad now with igniter installed")

while True:
    print("Confirm calibration or kill the script. Rocket should be on the launchpad at the proper angle now")
    userInput = input("Enter 2 to continue: ")  # Changed str(input()) to input() for Python 3
    if userInput == '2':
        break

# if we want to offset any IMU angle bias this is where we would do it....
heading, roll, pitch = bno.read_euler()
#######
if pitch > 0:
    pitchVal = 180 - pitch
else:
    pitchVal = -180 - pitch
desiredPitch = pitchVal      # this is where we want to point
desiredRoll = roll           # desired roll angle
print("Desired pitch = {0:0.2F} deg, and Desired roll = {1:0.2F} degrees".format(desiredPitch, desiredRoll))

while True:
    print("Confirm orientation or kill the script")
    userInput = input("Enter 2 to continue: ")  # Changed str(input()) to input() for Python 3
    if userInput == '2':
        break

print("Fire when ready - no more messages will be given.")

# wait for launch and then do control loop
maxAZ = -14      # this is not a light touch, but does require a bump up or down
# maxAZ = 20 #20 should not be used - this just bypasses the loop for testing purposes
gx = 0
gy = 0
gz = 0  # gyroscope values

while True:
    time.sleep(0.02)
    ax, ay, az = bno.read_accelerometer()
    with open("IMU.txt", "a") as f:
        f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(
            heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2))
    
    if az < maxAZ:
        # there is an event happening. to make sure its not jitter, check it again
        time.sleep(0.03)
        ax, ay, az = bno.read_accelerometer()
        with open("IMU.txt", "a") as f:
            f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(
                heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2))
    
    if az < maxAZ:
        # sustained. check it again
        time.sleep(0.03)
        ax, ay, az = bno.read_accelerometer()
    
    if az < maxAZ:
        with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
            f.write('Launch Detected\n')
            f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(
                heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2))
            g.write('Launch Detected\n')
            g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(
                heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2))
        maxAZ = az
        break

#### main control loop
time_old = current_milli_time()
time_start = time_old           # we are about 0.2seconds into the burn at this point
Kp = 45 * 3.14 / 180.
Kd = 10 * 3.14 / 180.
Ki = 0.1 * 3.14 / 180.
roll_integral = 0
roll_old = 0
servoYval = centerVal2
rollSetpoint = 0
pitch_old = 0
pitch_integral = 0
ejectionfirecheck = True      # this is a flag to see if we have fired the ejection charge

while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    gx, gy, gz = bno.read_gyroscope()
    ax, ay, az = bno.read_accelerometer()
    
    time_now = current_milli_time()
    deltaT = time_now - time_old
    
    # after 3s, the control loop does nothing so need to break
    if (time_now - time_start) > 55000:
        break

    if (time_now - time_start) > 4500 and ejectionfirecheck:  # optimal delay is about 1.9s after burn out, ~2.6s burn time after detecting launch
        ejectionfirecheck = False
        # fire!
        with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
            f.write("About to fire Ejection Charge - {0}\n".format(current_milli_time()))
            g.write("About to fire Ejection Charge - {0}\n".format(current_milli_time()))
            
            ###### MAKE SURE THIS IS ON FOR AN ACTUAL TEST !!!! #####
            GPIO.output(20, GPIO.HIGH)   # Firing ejection charge
            
            f.write("Fired Ejection Charge - {0}\n".format(current_milli_time()))
            g.write("Fired Ejection Charge - {0}\n".format(current_milli_time()))
    
    # pitch rotates about the x-axis gyroX is d(pitch)/dt
    # roll rotates about the y-axis. gyroY is d(roll)/dt
    
    roll_derivative = gy * 57.32       # 180/pi = 57.32. convert to degrees/s
    roll_integral = roll_integral + (roll - desiredRoll) * (deltaT / 1000)
    
    rollSetpoint = -Kp * (roll - desiredRoll) - Kd * (roll_derivative) - Ki * (roll_integral)
    servoYval = centerVal2 - rollSetpoint / 0.009     # map desired degrees to servo counts
    
    if servoYval > maxVal2:
        servoYval = maxVal2
    if servoYval < minVal2:
        servoYval = minVal2
    
    # pitch corresponds to rotation about x-axis on my rocket
    if pitch > 0:
        pitchVal = 180 - pitch
    else:
        pitchVal = -180 - pitch
    
    pitch_derivative = gx * 57.32        # 180/pi=57.32
    pitch_integral = pitch_integral + (pitchVal - desiredPitch) * (deltaT / 1000)
    pitchSetpoint = -Kp * (pitchVal - desiredPitch) - Kd * (pitch_derivative) - Ki * pitch_integral
    servoXval = centerVal1 - pitchSetpoint / 0.009
    
    if servoXval > maxVal1:
        servoXval = maxVal1
    elif servoXval < minVal1:
        servoXval = minVal1
    
    if servoYval > maxVal2:
        servoYval = maxVal2
    elif servoYval < minVal2:
        servoYval = minVal2
    
    if ejectionfirecheck:    # after ejection charge is fired, no need 
        pwm.set_servo_pulsewidth(servo2, servoYval)
        pwm.set_servo_pulsewidth(servo1, servoXval)
    
    time_old = time_now
    roll_old = roll
    pitch_old = pitchVal
    
    with open("IMU.txt", "a") as f:
        f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(
            heading, roll, pitchVal, ax, ay, az, gx, gy, gz, time_old, rollSetpoint, pitchSetpoint))

# exited the main control loop - motor is exhausted
pwm.set_servo_pulsewidth(servo1, centerVal1)
pwm.set_servo_pulsewidth(servo2, centerVal2)

with open("IMU.txt", "a") as f, open("IMUbackup.txt", "a") as g:
    f.write("-------------------------")
    g.write("-------------------------")
