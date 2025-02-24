#include "converter_djisdk_mav_node.hpp"


void spin_thread () {
    ros::spin();
}

class ConverterMAVtoDJI {

public:
    ConverterMAVtoDJI(const ros::Publisher& joy_pub, const ros::Publisher& odom_pub, const ros::Publisher& poseStamped_pub, double mass)
    {
        joy_pub_  = joy_pub;
        odom_pub_ = odom_pub;
        poseStamped_pub_ = poseStamped_pub;
        mass_ = mass;
    }


    void controlCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg){

        sensor_msgs::Joy joy_;
        //ROS_INFO("stacking1");
        joy_.axes.resize(5);
        joy_.axes[0] = msg->roll; //ERROR!
        //ROS_INFO("stacking2");
        joy_.axes[1] = msg->pitch;
        //ROS_INFO("stacking");
        auto g = Eigen::Vector3f(0,0,9.8066);
        joy_.axes[2] = msg->thrust.z/(mass_*2*g(2))*100.0;
        //ROS_INFO("stacking");
        joy_.axes[3] = msg->yaw_rate;
        //ROS_INFO("stacking");
        joy_.axes[4] = 0x28;
        //ROS_INFO("Trying to pub Joy");
        joy_pub_.publish(joy_);
        //ROS_INFO("Publised Joy");
    }

    void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
        //ROS_INFO_STREAM_THROTTLE(1, "velocityCallback" << msg->vector);
        Eigen::Quaterniond W_B_orientation = mav_msgs::quaternionFromMsg(odometry_.pose.pose.orientation);
        Eigen::Vector3d W_position = mav_msgs::vector3FromMsg(msg->vector);
        Eigen::Vector3d B_position =  W_B_orientation.inverse() * W_position;
        mav_msgs::vectorEigenToMsg(B_position, &odometry_.twist.twist.linear);
    }

    void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
        //ROS_INFO_STREAM_THROTTLE(1, "positionCallback" << msg->point);
        odometry_.pose.pose.position = msg->point;
    }

    void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
        //ROS_INFO_STREAM_THROTTLE(1, "attitudeCallback" << msg->quaternion);
        odometry_.pose.pose.orientation = msg->quaternion;
    }

    void publish_odometry(){
        odom_pub_.publish(odometry_);

        geometry_msgs::PoseStamped poseStamped_;
        poseStamped_.header = odometry_.header;
        poseStamped_.pose = odometry_.pose.pose;

        poseStamped_pub_.publish(poseStamped_);
    }

private:
    nav_msgs::Odometry odometry_;
    ros::Publisher joy_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher poseStamped_pub_;
    double mass_;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "converter_djisdk_mav_node");
  
    //Define node
    ros::NodeHandle nh, private_nh("~");

    double mass;
    if (!private_nh.getParam("mass", mass)) {
      ROS_ERROR("mass in converter_djisdk_mav is not loaded from ros parameter server");
      abort();
    }

    ROS_INFO("Inilizing publishers");
    //Define publishers
    ros::Publisher joy_pub  = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("ground_truth/odometry", 10);
    ros::Publisher poseStamped_pub = nh.advertise<geometry_msgs::PoseStamped>("ground_truth/pose",10);

    ConverterMAVtoDJI converter(joy_pub,odom_pub,poseStamped_pub, mass);

    ROS_INFO("Inilizing subscribers");

    //Define subscribers
    ros::Subscriber u_sub   = nh.subscribe<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 10, &ConverterMAVtoDJI::controlCallback, &converter);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 10, &ConverterMAVtoDJI::velocityCallback, &converter);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PointStamped>("dji_sdk/local_position", 10, &ConverterMAVtoDJI::positionCallback, &converter);
    ros::Subscriber att_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("dji_sdk/attitude", 10, &ConverterMAVtoDJI::attitudeCallback, &converter);
  
    //boost::thread sthread(spin_thread);

    //ros::spin();

    int ros_Hz = 50;
    int print_Hz = 1;
    ros::Rate loop_rate(ros_Hz);

    while (ros::ok()) {

        converter.publish_odometry();

        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_THROTTLE(1, "Looping");
    }

    return 0;
}
