/*
baxter_demos
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <cmath>
#include "table_setting_demo/log.h"
#include "table_setting_demo/table_object_behavior_VisionManip.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "table_setting_demo/table_setting_demo_types.h"
#include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/object_request.h"
#include "table_setting_demo/pick_and_place_state.h"
#include "table_setting_demo/object_position.h"
#include "table_setting_demo/pick_and_place_stop.h"
#include "table_setting_demo/ObjectTransformation.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <robotics_task_tree_msgs/ObjStatus.h>
#include <table_setting_demo/table_object_behavior_VisionManip_human.h>

namespace pr2 {
typedef enum STATE {
  APPROACHING = 0,
  PICKING,
  PICKED,
  PLACING,
  PLACED,
  NEUTRAL,
  IDLE
} STATE_t;
}  // namespace pr2

namespace task_net {

static const char *dynamic_object_str[] = {
  // "cup",
  // "bowl",
  // "soda",
  // "fork",
  // "spoon",
  // "knife"
};
static const char *static_object_str[] = {
  // // "fork",
  // // "spoon",
  // // "knife",
  // // //"cup",
  // // "bowl",
  // // "soda",
  // // "neutral",
  // // "placemat",
  // // "wineglass",
  // // "plate",
  // // // tkdjflkajsd;fnaskdf;
  // // "Cup",
  // // "Tea",
  // // "Sugar",
  // // "Left_Bread",
  // // "Right_Bread",
  // // "Meat",
  // // "Lettuce"
  // // DARS DEMO
  // "teddy_bear",
  // // "orange",
  // "sports_ball",
  // "clock",
  // // "bottle",
  // "scissors",
  // "cup",
  // // "bowl",
  "Cup",
  "Tea_Pot",
  "Sugar",
  "Burger",
  "Sandwich",
  "Apple",
  //"Orange"
};

TableObject_VisionManip_human::TableObject_VisionManip_human() : arm_group_{"right_arm"} { ROS_ERROR("START OF TableObject_VisionManip CONSTRUCTOR"); }
TableObject_VisionManip_human::TableObject_VisionManip_human(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    std::string mutex_topic,
    std::vector<float> pos,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : 
object_pos(pos), object_(object),
Behavior(name,
      peers,
      children,
      parent,
      state, 
      object),
       mut_arm(name.topic.c_str(), mutex_topic), nh_(), tf_listener_(),
       arm_group_{"right_arm"} {

  // flag saying whether the ROS publishers/listeners have been created

  ROS_ERROR("START OF TableObject_VisionManip CONSTRUCTOR@@");

  ready_to_publish_ = false;

  object_ = object;

   std::string topic = std::string("/") + object + std::string( "_status");
    //sprintf(topic, "/%s_status", object_.c_str());
    obj_status_sub_ = local_.subscribe(topic.c_str(), 1000, &TableObject_VisionManip_human::ObjStatusCallback, this );
  ROS_ERROR("END OF TableObject_VisionManip CONSTRUCTOR");

}
TableObject_VisionManip_human::~TableObject_VisionManip_human() {}

void TableObject_VisionManip_human::UpdateActivationPotential() {
  if( state_.done || parent_done_ || state_.peer_active || state_.peer_done )
  {
    ROS_DEBUG_THROTTLE_NAMED( 1, "HumanBehaviorTrace", "[%s]: State/Parent is done, so don't update activationpotential [%d|%d|%d|%d]", object_.c_str(), state_.done, parent_done_, state_.peer_active, state_.peer_done );
    state_.activation_potential = 0;
    return;
  }

  if( !IsActive() )
  {
    ROS_INFO_THROTTLE_NAMED( 1, "HumanBehaviorTrace", "[%s]: Not active, so don't update activationpotential", object_.c_str() );
    state_.activation_potential = 0;
    return;
  }
  state_.activation_potential = obj_chance_;

  // ROS_INFO("OBJ(%s): updating activation potential: %0.2f", object_.c_str(), state_.activation_potential);

  // ROS_INFO("object_pos: %f %f %f", object_pos[0],object_pos[1],object_pos[2]);
  // ROS_INFO("object_pos: %f %f %f", neutral_object_pos[0],neutral_object_pos[1],neutral_object_pos[2]);
  // ROS_INFO("x %f y %f z %f dist %f activation_potential %f", x, y, z, dist, state_.activation_potential);
}

/*void TableObject_VisionManip::PickAndPlace(std::string object) {
  table_setting_demo::pick_and_place msg;
  msg.request.object = object;
  if (ros::service::call("pick_and_place_object", msg)) {

  }
}*/
bool TableObject_VisionManip_human::Precondition() {
  if( obj_started_ == 1) {
    return true;
  }
  else{
    return false;
  }
}
bool TableObject_VisionManip_human::ActivationPrecondition() {
   bool lock_okay;

 // first attempt which did not work.....
  // check peer states...?
  if(!state_.peer_active && !state_.peer_done) {
    // return mut_arm.Lock(state_.activation_potential);
    ROS_INFO("\t[%s]: Trying to gain access to mutex!", name_->topic.c_str());
    lock_okay = mut_arm.Lock(state_.activation_potential);
    // return true;
  }
  else{
    ROS_INFO("\t[%s]: Peer gained access first, don't activate!", name_->topic.c_str());
    return false;
  }

// second attempt
  // check peer states...?
  if(state_.peer_active || state_.peer_done) {
    ROS_INFO("\t[%s]: Peer gained access first so release mutex and don't activate!", name_->topic.c_str());
    mut_arm.Release();
    return false;
  }

  return lock_okay;
}

bool TableObject_VisionManip_human::PickAndPlaceDone() {
  /*table_setting_demo::pick_and_place msg;
  msg.request.object = object_;
  ros::service::call("pick_and_place_check", msg);
  return msg.response.success;*/
}

void TableObject_VisionManip_human::Work() {
   ROS_INFO("HumanBehavior::Work: waiting for pause to be done!");
  // boost::this_thread::sleep(boost::posix_time::millisec(10000));
  // PickAndPlace(object_, robot_des_);
  //   // while (!PickAndPlaceDone()) {
  //     //boost::this_thread::sleep(boost::posix_time::millisec(500));
  //       // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
  //   // }
  // mut_arm.Release();

  while(obj_done_ == 0){
  ROS_WARN("[%s]: HumanBehavior::Working!", name_->topic.c_str());

    continue;
  }
  mut_arm.Release();
  state_.done = true;
  ROS_INFO("[%s]: HumanBehavior::Work: Done!", name_->topic.c_str());
}
 /*void TableObject_VisionManip_human::StateCallback( table_task_sim::SimState msg)
  {
    table_state_ = msg;
  }*/

  void TableObject_VisionManip_human::ObjStatusCallback( robotics_task_tree_msgs::ObjStatus msg)
  {
    // table_state_ = msg;
    obj_chance_ = msg.chance;
    obj_started_ = msg.started;
    obj_done_ = msg.done;

  }

}

