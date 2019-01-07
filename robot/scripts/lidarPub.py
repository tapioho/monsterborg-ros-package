#!/usr/bin/env python

from XVLidar import *
import rospy, time
from sensor_msgs.msg import LaserScan

global lidar, avgSpeed, T

# USB serial port info
port = "/dev/ttyACM0"  # Replace with the correct port name if needed
baudrate = 115200      # Symbol rate

# Setup LiDAR
lidar = XVLidar(port, baudrate)
lidar.setRPM(300)

# Calculate the average speed
print "Calculating average speed..."
avgSpeed = 0
iterations = 5
for i in range(iterations):
    rpm = lidar.getSpeed()
    avgSpeed += rpm
    rospy.sleep(0.1)
avgSpeed /= iterations
try:
    T = 1 / (avgSpeed / 60)   # Time (s) it takes for a single revolution
except:
    # Problem while reading speed
    T = 300 
print ("Average speed: %.2f \n" % avgSpeed)


def lidarToROS(data, avgSpeed=200):
    global T
    """Converts data from LiDAR into a LaserScan ROS message """
    # Calculate measurement increment time
    Tmeasure = T / 360  # A single revolution is divided into 360 measurements

    # Create a LaserScan message with the correct values
    msg = LaserScan()
    msg.header.stamp = rospy.get_rostime()
    #msg.header.frame_id = "laser_frame"
    msg.angle_min = 0.0
    msg.angle_max = 359.0
    msg.time_increment = Tmeasure
    msg.angle_increment = 1.0
    msg.scan_time = 1.0
    msg.range_min = 0.15
    msg.range_max = 6.0

    # Store the data into LaserScan message
    for i in range(data.shape[0]):
        msg.ranges.append(data[i,0])
        msg.intensities.append(data[i,1])

    return msg

def lidarScan():
    global lidar, avgSpeed, T
    lidar.scan(2*T)
    data = lidar.getData()
    msg = lidarToROS(data, avgSpeed)
    return msg

def lidarPublisher():
    #global lidar
    # Create and initialize a  publisher
    pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rospy.init_node('lidar', anonymous=True)
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        try:
            msg = lidarScan()
	    print(msg.header.stamp)
            pub.publish(msg)
        except KeyboardInterrupt:
            print "Interrupted by user"
            break
        except:
            print("Error: " + str(sys.exc_info()))
            lidar.reset()
	    lidar.setRPM(300)

            #break
        rate.sleep()

if __name__ == '__main__':
    try:
        lidarPublisher()

    except rospy.ROSInterruptException:
        pass










