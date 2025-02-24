
#include "ros/ros.h"
#include <ros/console.h>
#include "common.h"

#include <XmlRpcException.h>

#include <scripted_motion_factory.h>

/*
This file construcs a path given the arguments given and publishes them in sequence on the 
/name/pose topic. The arguments are:
- motion: the type of motion to be generated
- name: the name of the model
- x, y, z: the initial position of the model
- hz: the frequency at which the poses are published
- arg: the arguments to the motion type, as a dictionary. The arguments are specific for a certain motion type and makes up a route
- vel exists also but does not work.

https://gitlab.liu.se/srg/srg/-/tree/master/catkin_ws/src/simulator-specifications.
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "scenario_path_generation");

  ros::NodeHandle nh;

  ScriptedMotionFactory motion_factory;

  std::string motion_name;
  searchGetParam("motion", motion_name);
  ScriptedMotion * motion = motion_factory.create(motion_name);
  if(motion == 0) {
    return 0;
  }

  std::string model_name;
  searchGetParam("name", model_name);
  motion->set_model_name(model_name);

  geometry_msgs::Pose pose;
  searchGetParam("x", pose.position.x);
  searchGetParam("y", pose.position.y);
  searchGetParam("z", pose.position.z);
  geometry_msgs::PoseStamped translation;
  translation.pose = pose;
  motion->set_translation(translation);
  
  XmlRpc::XmlRpcValue arguments;
  searchGetParam("arg", arguments, XmlRpc::XmlRpcValue());
  try {
    motion->set_parameters(arguments);
  } catch(const XmlRpc::XmlRpcException &e) {
    ROS_ERROR_STREAM("Invalid arguments to motion type '" << motion->get_model_name() << "': " << arguments << ", Exception " << e.getCode() << ": " << e.getMessage());
  }

  double hz;
  searchGetParam("hz", hz, 100.0);

  ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/"+motion->get_model_name()+"/pose/", 100);

  ROS_INFO_STREAM("Scenario path generation for model '" << motion->get_model_name() << "' online");

  ros::Rate spin_rate(hz);

  while(ros::ok())
  {
    ros::spinOnce();

    geometry_msgs::PoseStamped pose = motion->generate_next();
    pose_publisher.publish(pose);

    spin_rate.sleep();
  }

  return 0;
}
