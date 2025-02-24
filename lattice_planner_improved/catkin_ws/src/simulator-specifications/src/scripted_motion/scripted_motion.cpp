#include "scripted_motion.h"

ScriptedMotion::ScriptedMotion() {
  startTime = ros::Time::now();
  time = startTime;
}

void ScriptedMotion::set_translation(geometry_msgs::PoseStamped translation) {
  translation_ = translation;
}


void ScriptedMotion::set_parameters(XmlRpc::XmlRpcValue parameters) {
  parameters_ = parameters;
  update_parameters();
}

void ScriptedMotion::set_model_name(std::string name) {
  model_name_ = name;
}



float s2f(std::string str) {
  std::stringstream ss;
  ss << str;
  float value;
  ss >> value;
  return value;
}

