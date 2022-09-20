#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from robotics_task_tree_msgs.msg import ControlMessage
from sound_play.libsoundplay import SoundClient
# from GoogleSpeechToText import *
# sound_handle=SoundClient()

def main_publisher():
    pub1 = rospy.Publisher('/launch_1', String, queue_size=100)
    pub2 = rospy.Publisher('/launch_2', String, queue_size=10)
    pub4 = rospy.Publisher('/launch_4', String, queue_size=10)
    root_topic = rospy.get_param("/topic")
    root_topic = str(root_topic)+"_parent"
    topic_activate_pub_ = rospy.Publisher(root_topic, ControlMessage, queue_size=100)
    # rospy.init_node('main_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    rate.sleep()
    while not rospy.is_shutdown():
        pub1.publish('table_task_sim table_task_sim.launch')
        pub2.publish('table_setting_demo multi_robot_task_demo_visionManip_baxter_param.launch')
        pub4.publish('remote_mutex table_setting_mutex_baxter.launch')

        # sound_handle.say("Thank you. Please help me to assemble the table top with me.","voice_cmu_us_clb_arctic_clunits")

        topic_activate_msg_ = ControlMessage()
        topic_activate_msg_.activation_level = 10000000000000000000000000000000.0
        topic_activate_pub_.publish(topic_activate_msg_)
        # rospy.spin()
        rate.sleep()
    # if __name__ == '__main__':
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass


