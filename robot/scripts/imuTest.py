#!/usr/bin/env python

import sys, getopt

sys.path.append('.')
import RTIMU
import os.path, time, math, operator, os

IMU_IP = "127.0.0.2"
IMU_PORT = 5005

MON_IP = "127.0.0.5"
MON_PORT = 5005

SETTINGS_FILE = "RTIMULib"

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)

# Offsets
yawoff = 0.0
pitchoff = 0.0
rolloff = 0.0

# Timers
t_print = time.time()  # Timestamp for latest print
t_calc = time.time()  # Timestamp for latest position calculation
t_fail = time.time()  # Timestamp for latest failure
t_fail_timer = 0.0
t_shutdown = 0  # Shutdown counter

# Try initializing
if not imu.IMUInit():
    tracker = time.time()
    imu_sentence = "$IIXDR, IMU_FAILED_TO_INITIALIZE*7C"
    if tracker-t_print > 1.0:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(imu_sentence, (IMU_IP, IMU_PORT))
        t_print = tracker
        t_shutdown += 1
        # Exit if initializing failed more than 9 times
        if t_shutdown > 9:
            sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)  # Enable gyro
imu.setAccelEnable(True)  # Enable accelerometer
imu.setCompassEnable(True)  # Enable compass

poll_interval = imu.IMUGetPollInterval()

# Data variables
roll = 0.0  # Rotation i.r.t x-axis
pitch = 0.0  # Rotation i.r.t y-axis
yaw = 0.0  # Rotation i.r.t z-axis
heading = 0.0  # Direction the nose of the vehicle is pointing to
rollrate = 0.0
pitchrate = 0.0
yawrate = 0.0
magnetic_deviation = -13.7

# Calculation variables
t_one = 0
t_three = 0
roll_total = 0.0
roll_run = [0] * 10
heading_cos_total = 0.0
heading_sin_total = 0.0
heading_cos_run = [0] * 30
heading_sin_run = [0] * 30


while True:
    tracker = time.time()

    # If it's been longer than 5 seconds since last data print
    if tracker-t_calc > 5.0:
        # If it's been more than 1 second since last failure
        if tracker-t_fail > 1.0:
            # Init dampening variables
            t_one = 0
            t_three = 0
            roll_total = 0.0
            roll_run = [0] * 10
            heading_cos_total = 0.0
            heading_sin_total = 0.0
            heading_cos_run = [0] * 30
            heading_sin_run = [0] * 30
            
            # Inform that a failure has happened
            print("Encountered a failure while reading IMU."  + str(round(t_fail_timer / 60, 1)))
            
            t_fail_timer += 1

            # Update timer and counter
            t_fail = tracker
            t_shutdown += 1

    # Check if new data is found
    if imu.IMURead():
        data = imu.getIMUData()
        fusionPose = data["fusionPose"]
        Gyro = data["gyro"]
        t_fail_timer = 0.0

        if tracker-t_calc > .1:
            roll = round(math.degrees(fusionPose[0]) - rolloff, 1)
            pitch = round(math.degrees(fusionPose[1]) - pitchoff, 1)
            yaw = round(math.degrees(fusionPose[2]) - yawoff, 1)
            rollrate = round(math.degrees(Gyro[0]), 1)
            pitchrate = round(math.degrees(Gyro[1]), 1)
            yawrate = round(math.degrees(Gyro[2]), 1)

            if yaw < 0.1:
                yaw += 360.0
            if yaw > 360.0:
                yaw -= 360.0

            # Position calculation
            roll_total -= roll_run[t_one]
            roll_run[t_one] = roll
            roll_total += roll_run[t_one]
            roll = round(roll_total / 10, 1)

            heading_cos_total -= heading_cos_run[t_three]
            heading_sin_total -= heading_sin_run[t_three]
            heading_cos_run[t_three] = math.cos(math.radians(yaw))
            heading_sin_run[t_three] = math.sin(math.radians(yaw))
            heading_cos_total += heading_cos_run[t_three]
            heading_sin_total += heading_sin_run[t_three]

            yaw = round(math.degrees(math.atan2(heading_sin_total/30, heading_cos_total/30)), 1)

            if yaw < 0.1:
                yaw += 360.0

            # Yaw is magnetic heading, convert to true heading (direction the nose is pointing)
            heading = yaw - magnetic_deviation
            if heading < 0.1:
                heading += 360.0
            if heading > 360.0:
                heading -= 360.0
                
            # Update timer
            t_calc = tracker
            
            t_one += 1
            if t_one == 10:
                t_one = 0
            t_three += 1
            if t_three == 30:
                t_three = 0

            # If it's been longer than 1 second since last print
            if tracker-t_print > 1.0:
                print "------------------------------------------------------------------"
                print("Heading: " + str(heading) + "      Roll: " + str(roll) + "     Pitch: " + str(pitch) + "      Yaw: " + str(yaw))
                print(data['accel'])

                # To IMU bus
                with open('imu_bus', 'w') as f:
                    f.write(str(t_print) + ',' + str(heading) + ',' + str(roll) + ',' + str(pitch))

                t_print = tracker

        time.sleep(poll_interval*1.0 / 1000.0)
                    
            

            



