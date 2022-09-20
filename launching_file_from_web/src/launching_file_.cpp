#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


    void callback_4(std_msgs::String msg){
        std::stringstream ss;
        ss << "roslaunch " << msg.data;
        std::system(ss.str().c_str());//std::system only accept c_str() a.k.a char*
    }


    int main(int argc, char** argv){
        ros::init(argc,argv,"roslaunch_launcher_dialog");
        ros::NodeHandle n;
       
	ros::Subscriber sub_2 = n.subscribe("/launch_dialog", 1, callback_4);
	

        ros::spin();
        return 0;
    }
