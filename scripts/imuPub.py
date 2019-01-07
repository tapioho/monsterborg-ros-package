#!/usr/bin/env python

import sys, time
import threading
import rospy
from sensor_msgs.msg import Imu
import RTIMU

def imuInit():
    global imu_instance, settings  # For some reason settings needs to be global...
    
    SETTINGS_FILE = "RTIMULib"

    # Set a new imu instance
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu_instance = RTIMU.RTIMU(settings)
    
    fail_counter = 0
    t_print = time.time() 
    
    # Try initializing
    print("Initializing IMU...")
    if not imu_instance.IMUInit():
        tracker = time.time()
        if tracker-t_print > 1.0:
            print("Failed to initialize, retrying...")
            t_print = tracker
            fail_counter += 1
            # Exit if initializing failed more than 9 times
            if fail_counter > 9:
                print("Initializing error, exiting...")
                sys.exit(1)
    
    imu_instance.setSlerpPower(0.02)
    imu_instance.setGyroEnable(True)  # Enable gyro
    imu_instance.setAccelEnable(True)  # Enable accelerometer
    imu_instance.setCompassEnable(True)  # Enable compass
    
    poll_interval = imu_instance.IMUGetPollInterval()
    time.sleep(poll_interval*1.0 / 1000.0)
    print("Done!\n")

def imuCap():
    global imu_instance  # global settings are not required here??
    global data
    
    # Read data
    print("Reading IMU")
    poll_interval = imu_instance.IMUGetPollInterval()
    
    while not rospy.is_shutdown():
        try:
            # Init to clear the data pipeline
            #imu_instance.IMUInit()
            time.sleep(poll_interval*2.0 / 1000.0)
            
            # Check if new data is found
            if imu_instance.IMURead():
                data = imu_instance.getIMUData()
                #fusionPose = data["fusionPose"]
                #Gyro = data["gyro"]
                #acceleration = data["accel"]
                #break
            else:
                # Set data to zero-values if reading wasn't succesful
                data = {"accel":(0.0, 0.0, 0.0), "gyro":(0.0, 0.0, 0.0)}
        except:
            print("Error: " + str(sys.exc_info()))
            break
    
def imuPublisher():
    global data
    # Create and initialize a publisher
    pub = rospy.Publisher('imuOutput', Imu, queue_size=1)
    rospy.init_node('imu', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    # Create a Imu message 
    msg = Imu()

    # Try reading data
    while not rospy.is_shutdown():
        try:
            #imuCap()
            msg.header.stamp = rospy.Time.now()
            # Save data to ROS message
            msg.linear_acceleration.x = data["accel"][0]
            msg.linear_acceleration.y = data["accel"][1]
            msg.linear_acceleration.z = data["accel"][2]
            msg.angular_velocity.x = data["gyro"][0]
            msg.angular_velocity.y = data["gyro"][1]
            msg.angular_velocity.z = data["gyro"][2]
            # Publish
            pub.publish(msg)
        except:
            print("Error: " + str(sys.exc_info()))
        rate.sleep()

    print("Exiting imu publisher...")
    sys.exit()
        
    
# Main program with two threads: imu capture and ROS-publisher
# ROS-publisher is required to be set as a master thread
if __name__ == "__main__":
    imuInit()
    t1 = threading.Thread(target=imuCap)
    t1.start()
    imuPublisher()
    
    
                        
    


