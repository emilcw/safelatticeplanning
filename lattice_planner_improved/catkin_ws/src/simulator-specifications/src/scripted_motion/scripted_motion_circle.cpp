#include "scripted_motion_circle.h"

geometry_msgs::PoseStamped ScriptedMotionCircle::generate_next() {
  float sec = ros::Time::now().toSec();

  geometry_msgs::Pose translation = translation_.pose;
  geometry_msgs::Pose pose;
  pose.position.z = translation.position.z + 5.0;
  pose.position.x = translation.position.x + radius_*std::cos(2.0*M_PI*0.2*sec);  // 5 seconds per revolution
  pose.position.y = translation.position.y + radius_*std::sin(2.0*M_PI*0.2*sec);  // 5 seconds per revolution
  
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose = pose;
  return poseStamped;
}


void ScriptedMotionCircle::update_parameters() {
  if(parameters_.hasMember("r")) {
    if(parameters_["r"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      radius_ = (int(parameters_["r"]));
    } else {      
      radius_ = double(parameters_["r"]);
    }
  }
}
