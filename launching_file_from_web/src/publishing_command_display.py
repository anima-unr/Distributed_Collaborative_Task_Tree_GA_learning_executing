#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


def main_baxter_display():
    pub1 = rospy.Publisher('/launch_display', String, queue_size=100)
 
    rospy.init_node('main_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    rate.sleep()
    print("hello")
    while not rospy.is_shutdown():
        pub1.publish('/home/bashira/catkin_ws/baxterworking_text.jpg')

        # rospy.spin()
        rate.sleep()
    # if __name__ == '__main__':
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass


