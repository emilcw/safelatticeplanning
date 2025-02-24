// Simple ROS motion simulator for the DJI Matrice M100, partial implementation of the DJI ROS interface http://wiki.ros.org/dji_sdk
// @author Olov Andersson <olov.a.andersson@liu.se>

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <assert.h>
//#include "sensor_msgs/NavSatFix.h"
//#include "std_msgs/UInt8.h"
//#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
//#include "geoconvert.h"
#include <map>
#include <string>
#include "model_matrice_m100.hpp"
//TODO: Obey these.
// MATRICE SDK state update rates
// attitude = 100Hz
// battery_state = 10Hz
// flight_status = 50Hz
// gimbal_angle = 50Hz
// gps_health = 50Hz
// gps_position = 50Hz
// height_above_takeoff = 50Hz
// imu = 100Hz
// rc = 50Hz
// velocity = 50Hz

mm100::VectorNU u_t(0,0,0); // Current control signals
mm100::VectorNX x_t = mm100::VectorNX::Zero(); // Current state

// Reset the m100 model to initial state with specific pose
bool resetPose = 0;
geometry_msgs::PoseStamped resetPoseMsg;

void spin_thread () {
    ros::spin();
}

void resetCallback(const geometry_msgs::PoseStamped msg) {
  resetPose = 1;
  resetPoseMsg = msg;
}

void flight_control_RPYrTCallback(const sensor_msgs::Joy::ConstPtr& pMsg) {
/*  uint8_t flag = (DJI::OSDK::Control::VERTICAL_VELOCITY |
                  DJI::OSDK::Control::HORIZONTAL_VELOCITY |
                  DJI::OSDK::Control::YAW_RATE |
                  DJI::OSDK::Control::HORIZONTAL_GROUND |
                  DJI::OSDK::Control::STABLE_ENABLE);
*/
  float roll        = pMsg->axes[0];
  float pitch       = pMsg->axes[1];
  float thrust      = pMsg->axes[2]; // NOTE: As per docs for /dji_sdk/flight_control_setpoint_generic in http://wiki.ros.org/dji_sdk , conflicts with RC topic order??
  float yaw_rate    = pMsg->axes[3];
  float flag        = pMsg->axes[4];
  
  //ROS_INFO("flight_control_RPYrTCallback");

  u_t(mm100::RREF) = roll;
  u_t(mm100::PREF) = pitch;
  u_t(mm100::THR) = thrust; // TODO: From where??
  // TODO: Add yaw control channel
  //assert(pMsg.flags==0x28); // TODO: Instate this check?
}

geometry_msgs::QuaternionStamped extract_attitude_msg(mm100::VectorNX x) {
    geometry_msgs::QuaternionStamped q;
    q.header.frame_id = "/world"; 
    q.header.stamp = ros::Time::now();
    tf::Quaternion tf_q;
//    tf_q.setRPY(x(mm100::roll), x(mm100::pitch), x(mm100::yaw));
//    q.quaternion = tf_q;
    q.quaternion = tf::createQuaternionMsgFromRollPitchYaw(x(mm100::ROLL), x(mm100::PITCH), x(mm100::YAW));
    return q;
}

geometry_msgs::Vector3Stamped extract_velocity_msg(mm100::VectorNX x) {
    geometry_msgs::Vector3Stamped v;
    v.header.frame_id = "/world";
    v.header.stamp = ros::Time::now();
    v.vector.x = x(mm100::VX);
    v.vector.y = x(mm100::VY);
    v.vector.z = x(mm100::VZ);
    return v;
}

geometry_msgs::PointStamped extract_position_msg(mm100::VectorNX x) {
    geometry_msgs::PointStamped p;
    p.header.frame_id = "/world";
    p.header.stamp = ros::Time::now();
    p.point.x = x(mm100::PX);
    p.point.y = x(mm100::PY);
    p.point.z = x(mm100::PZ);
    return p;
}

geometry_msgs::PoseStamped extract_pose_msg(mm100::VectorNX x) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "/world";
  p.header.stamp = ros::Time::now();
  p.pose.position.x = x(mm100::PX);
  p.pose.position.y = x(mm100::PY);
  p.pose.position.z = x(mm100::PZ);
  p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(x(mm100::ROLL), x(mm100::PITCH), x(mm100::YAW));
  return p;
}


/*
bool sdkCtrlAuthorityCallback(dji_sdk::SDKControlAuthority::Request&  request,
			      dji_sdk::SDKControlAuthority::Response& response) {

  ROS_DEBUG("called sdkCtrlAuthorityCallback");

  //  response.cmd_set  = (int)ack.info.cmd_set;
  //  response.cmd_id   = (int)ack.info.cmd_id;
  //  response.ack_data = (unsigned int)ack.data;

  response.cmd_set  = 0;
  response.cmd_id   = 0;
  response.ack_data = 0;
  response.result = true;

  return true;
}

bool droneTaskCallback(dji_sdk::DroneTaskControl::Request&  request,
		       dji_sdk::DroneTaskControl::Response& response) {

  ROS_DEBUG("called droneTaskCallback");

  ROS_INFO("droneTaskCallback: %d", request.task);


  if (request.task == 4)
  {
    // takeoff
    sim->takeoff();
    ROS_DEBUG("called vehicle->control->takeoff()");
  }
  else if (request.task == 6)
  {
    // landing
    sim->land();
    ROS_DEBUG("called vehicle->control->land()");
  }
  else if (request.task == 1)
  {
    // gohome
    ROS_DEBUG("called vehicle->control->goHome()");
  }
  else
  {
    ROS_WARN("unknown request task in droneTaskCallback");
    response.result = false;
  }

  response.cmd_set  = 0;
  response.cmd_id   = 0;
  response.ack_data = 0;
  response.result = true;

  return true;
}
*/

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "motion_sim_matrice_m100");
  
  ros::NodeHandle nh;
  
  ros::Publisher attitude_publisher =
    nh.advertise<geometry_msgs::QuaternionStamped>("dji_sdk/attitude", 10); // TODO: Verify controller and hardware simulator has same control domain.

  ros::Publisher velocity_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 10); // TODO: Verify this is the correct velocity topic for the Matrice.

  ros::Publisher position_publisher =
    nh.advertise<geometry_msgs::PointStamped>("dji_sdk/local_position", 10); // TODO: Might use gps_position with Tommy's conversion instead for compatiblity.
//    nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10);

  ros::Publisher odometry_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/odometry", 10); // TODO: Verify this is the correct odometry topic for the Matrice.

  ros::Publisher pose_publisher =
    nh.advertise<geometry_msgs::PoseStamped>("dji_sdk/local_pose", 10);
/*
  ros::ServiceServer sdk_ctrlAuthority_server  =
    nh.advertiseService("dji_sdk/sdk_control_authority", sdkCtrlAuthorityCallback); // TODO: Controller must set up and simulator verify authority

  ros::ServiceServer drone_task_server =
    nh.advertiseService("dji_sdk/drone_task_control", droneTaskCallback); // TODO: Controller must set up and simulator verify takeoff status etc
*/

  double x0 = 0.0;
  double y0 = 0.0;
  double z0 = 0.0;
  ros::param::get("/x", x0);
  ros::param::get("/y", y0);
  ros::param::get("/z", z0); // TODO: Must match Tommy's  / on-board initial pose.

  ros::Subscriber flight_control_RPYrT =
    nh.subscribe<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10,
				   flight_control_RPYrTCallback);
				     
  ros::Subscriber sim_reset =
    nh.subscribe<geometry_msgs::PoseStamped>("sim_pose_reset", 10, resetCallback);
  
  boost::thread sthread(spin_thread);
  
  int ros_Hz = 1000;
  int physics_Hz = 100; 
  int attitude_msg_Hz = 100;
  int velocity_msg_Hz = 50;
  int position_msg_Hz = 50;
  int print_Hz = 2;
  const ros::Duration physics_duraction(1.0/physics_Hz);
  const ros::Duration attitude_duraction(1.0/attitude_msg_Hz);
  const ros::Duration velocity_duraction(1.0/velocity_msg_Hz);
  const ros::Duration position_duraction(1.0/position_msg_Hz);
  const ros::Duration print_duraction(1.0/print_Hz);

  std::map<std::string,ros::Time> time_staps;
  ros::Time now = ros::Time::now();
  time_staps["physics_update"] =  now;
  time_staps["attitude_update"] = now;
  time_staps["velocity_update"] = now;
  time_staps["position_update"] = now;
  time_staps["print_update"] = now;



  // Set up motion model 
  using namespace mm100;
  VectorNX start_state = VectorNX::Zero();
  std::cerr << x0 << " : " << y0 << " : " << z0 << std::endl;
  start_state(PX) = x0;
  start_state(PY) = y0;
  start_state(PZ) = z0;
  MotionM100 sim(start_state, 1.0/physics_Hz);

  while (ros::ok()) {
    now = ros::Time::now();

    if (now - time_staps["attitude_update"] >= attitude_duraction) {
      // std::cerr << "attiude = " <<  1.0/( now - time_staps["attiude_update"] ).toSec() << std::endl;
      time_staps["attitude_update"] = now;
      attitude_publisher.publish(extract_attitude_msg(x_t));
    }
    // std::cerr <<  now <<  std::endl;


    if (now - time_staps["physics_update"] >= physics_duraction) {
      // std::cerr << "physics = " <<  1.0/( now - time_staps["physics_update"] ).toSec() << std::endl;
      time_staps["physics_update"] = now;
      // Tick simulator
      x_t = sim.tick(u_t, false, z0);
      
      // Handle reset of pose
      if(resetPose) {
        resetPose = 0;
        ROS_ERROR_STREAM("resetPose == 1");
        VectorNX reset_state = VectorNX::Zero();
  
        reset_state(PX) = resetPoseMsg.pose.position.x;
        reset_state(PY) = resetPoseMsg.pose.position.y;
        reset_state(PZ) = resetPoseMsg.pose.position.z;
        
        /* TODO: extract roll, pitch, yaw from msg and use
        reset_state(mm100::ROLL) = resetPoseMsg.pose.;
        reset_state(mm100::PITCH) = resetPoseMsg.pose.;
        reset_state(mm100::YAW) = resetPoseMsg.pose.;
        sim.reset(reset_state);
        */
        sim.reset(reset_state);
        ROS_ERROR_STREAM("resetted to (x,y,z): " << resetPoseMsg.pose.position.x << ", " 
                                                << resetPoseMsg.pose.position.y << ", " 
                                                << resetPoseMsg.pose.position.z);
      }
    }


    if (now - time_staps["velocity_update"] >= velocity_duraction) {
      // std::cerr << "velocity = " <<  1.0/( now - time_staps["velocity_update"] ).toSec() << std::endl;
      time_staps["velocity_update"] = now;
      velocity_publisher.publish(extract_velocity_msg(x_t));
    }

    if (now - time_staps["position_update"] >= position_duraction) {
      // std::cerr << "position = " <<  1.0/( now - time_staps["position_update"] ).toSec() << std::endl;
      time_staps["position_update"] = now;

      position_publisher.publish(extract_position_msg(x_t));
      pose_publisher.publish(extract_pose_msg(x_t));
    }

    // TODO: Implement GPS coords and flight modes?
/*
    if (% 100 == 0) {
      gps_position_publisher.publish(sim->get_gps_position());
    }
    
    if (% 100 == 0) {
      gps_health_publisher.publish(sim->get_gps_health());
    }
    
    if (% 100 == 0) {
      flight_status_publisher.publish(sim->get_flight_status());
    }

    if (% 100 == 0) {
      height_publisher.publish (sim->get_height_above_takeoff ());
    }
*/    
    
    if (now - time_staps["print_update"] > print_duraction) {
      std::cerr << now <<  1.0/( now - time_staps["print_update"] ).toSec() << " c=" <<  ", x_t=[" << x_t.transpose() << "], u_t=[" << u_t.transpose() << "]." << std::endl;
      time_staps["print_update"] = now;
    }
    // std::cerr <<  ros::Time::now() - now  << std::endl;
  }

  return 0;
}
