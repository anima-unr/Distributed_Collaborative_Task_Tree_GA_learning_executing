// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <sstream>


//     void callback_4(std_msgs::String msg){
//         std::stringstream ss;
//         ss << "roslaunch " << msg.data;
//         std::system(ss.str().c_str());//std::system only accept c_str() a.k.a char*
//     }


//     int main(int argc, char** argv){
//         ros::init(argc,argv,"roslaunch_launcher_hand_intention");
//         ros::NodeHandle n;
       
// 	ros::Subscriber sub_2 = n.subscribe("/launch_hand_intention", 1, callback_4);
	

//         ros::spin();
//         return 0;
//     }

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "Poco/Process.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>


Poco::ProcessHandle* ph;
bool running = false;

void callback_run(std_msgs::String msg){
    if(!running){
        std::vector<std::string> args;
        boost::split(args, msg.data, boost::is_any_of(" ") ); //Split the msg.data on space and save it to a vector
        Poco::ProcessHandle ph_running = Poco::Process::launch("roslaunch", args,0,0,0); //launch a new node
        ph = new Poco::ProcessHandle(ph_running); // Copy the processhandler to our global variable
        running = true; //Refuse new launch
        ROS_INFO_STREAM("launched : roslaunch" << msg.data);
    }
    else{
        ROS_ERROR("A process is already running.");
    }
}

void callback_kill(std_msgs::Empty msg){
    if(running){
        Poco::Process::requestTermination(ph->id()); //send SIGINT
            Poco::Process::wait(*ph); //Wait for roslaunch to kill every node
        free(ph);  
        running = false;  //accept a new launch
        ROS_INFO("Killed process");
    }
    else{
        ROS_ERROR("No Process are running.");
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "roslaunch_launcher_hand_intention");
    ros::NodeHandle n;
    ros::Subscriber sub_run = n.subscribe("/launch_hand_intention",100,callback_run);
    ros::Subscriber sub_kill = n.subscribe("/kill_hand_intention",100,callback_kill);
    ros::spin();
    return 0;
}