#!/usr/bin/env python
# Copyright (c) 2016 Rethink Robotics, Inc.

# rospy for the subscriber
import rospy
from cv2 import cv2
# ROS Image message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

pub = rospy.Publisher("/robot/xdisplay", Image, queue_size=10)

def main_display(img_file):

    # Define your image topic
    # rospy.init_node('image_converter', anonymous=True)
    # images="/home/bashira/catkin_ws_old/src/baxter_examples/share/images/baxterworking.png"
    # Define the Display topic

    # Set up your subscriber and define its callback
    # pub = rospy.Publisher(xdisplay_topic, Image, queue_size=10)
    cv_image = cv2.imread(img_file,1)

    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    pub.publish(image_message)
    rospy.spin()
    
    # Spin until ctrl + c
if __name__ == '__main__':
    main_display("/home/bashira/catkin_ws/baxterworking_text.jpg")



