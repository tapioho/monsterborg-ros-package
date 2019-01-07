#!/usr/bin/env python

import sys
import picamera
from picamera.array import PiRGBArray
from time import sleep
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import rospy


RESOLUTION = (640, 480)
FPS = 10

# Initialize the camera
global cam
cam = picamera.PiCamera()
cam.rotation = 0
cam.resolution = RESOLUTION

# Initialize PiRGBArray object
global rawCap
rawCap = PiRGBArray(cam, size=RESOLUTION)

# Allow the camera to warmup
sleep(1)

# Camera capture
def cameraCap():
    global cam, rawCap
    # Capture frames
    try:
        for frame in cam.capture_continuous(rawCap, format='bgr', use_video_port=True):
            # Grab the raw numpy array (image)
            img = frame.array
            # Clear the stream to prepare for the next frame
            rawCap.truncate(0)
            # Encode and convert to CompressedImage message
            compImg = np.array(cv2.imencode('.jpg', img)[1]).tostring()          
            return compImg
    except:
        print("Error while reading camera feed: " + str(sys.exc_info()))

# Publisher
def camPublisher():
    # Create and initialize a publisher 
    pub = rospy.Publisher('/cameraOutput', CompressedImage, queue_size=1)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(FPS)
    
    # Create a CompressedImage message
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    while not rospy.is_shutdown():
        try:
            msg.data = cameraCap()
            pub.publish(msg)
        except:
            print("Error while publishing camera feed: " + str(sys.exc_info()))
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        camPublisher()
        cam.close()
        
    except rospy.ROSInterruptException:
        pass



