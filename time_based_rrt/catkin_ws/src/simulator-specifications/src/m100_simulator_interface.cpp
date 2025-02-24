#include "m100_simulator_interface.h"

std::string model_name;
std::string reference_frame;

geometry_msgs::Pose target_pose;
geometry_msgs::Twist target_twist;

gazebo_msgs::ModelState target_model_state;
gazebo_msgs::LinkState target_gimbal_state;

ros::Subscriber twist_subscriber;
ros::Subscriber pose_subscriber;
ros::Subscriber attitude_quaternion_subscriber;

ros::ServiceClient model_state_client;
gazebo_msgs::SetModelState set_model_state;
gazebo_msgs::SetLinkState set_link_state;

bool keep_state;
bool is_state_updated = true;
bool velocity_updated = false;
bool position_updated = false;

void twistCallback(const geometry_msgs::Twist msg)
{
  target_twist = msg;
  velocity_updated = true;
  is_state_updated = true;
}

void poseCallback(const geometry_msgs::PoseStamped msg)
{
  target_pose = msg.pose;
  position_updated = true;
  is_state_updated = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "simulator_interface_m100");

  //target_pose.position.z = 1.0; //Trying to keep the drone at 1m height

  ros::NodeHandle nh;

  searchGetParam("name", model_name, std::string("m100_100"));
  searchGetParam("frame", reference_frame, std::string("world"));
  searchGetParam("keep_state", keep_state, true);
  searchGetParam("x", target_pose.position.x, 0.0);
  searchGetParam("y", target_pose.position.y, 0.0); 
  searchGetParam("z", target_pose.position.z, 1.0);

  pose_subscriber = nh.subscribe("/"+model_name+"/pose/", 1000, poseCallback);
  twist_subscriber = nh.subscribe("/"+model_name+"/vel/", 1000, twistCallback);

  model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);

  ROS_INFO_STREAM("Bridge between PC sim and gazebo connected");
  ROS_INFO_STREAM("Steering model to (x,y,z) = " << "(" << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << ")");

  ros::Rate spin_rate(200);

  while(ros::ok())
  {
    ros::spinOnce(); // check for incoming messages

    if(model_state_client) //If contact with Gazebo
    {
      if(keep_state || is_state_updated) //Om keep_state är true eller om vi har en ny state (Första gången är både true, andra gången är bara en true)
      {
        target_model_state.model_name = model_name;
        target_model_state.reference_frame = reference_frame;
        target_model_state.pose = target_pose;
        target_model_state.twist = target_twist;
        set_model_state.request.model_state = target_model_state;
        model_state_client.call(set_model_state);
        is_state_updated = false;
      }
    }
    else
    {
      ROS_WARN_STREAM("Update model state failed");
    }

    spin_rate.sleep();
  }

  return 0;
}
