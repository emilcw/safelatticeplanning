#ifndef __SCRIPTED_MOTION_H__
#define __SCRIPTED_MOTION_H__

#include "ros/ros.h"
#include <ros/console.h>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>

class ScriptedMotion
{
  public:
    ScriptedMotion();
    void set_model_name(std::string name);
    void set_translation(geometry_msgs::PoseStamped translation);
    void set_parameters(XmlRpc::XmlRpcValue parameters);
    std::string get_model_name() { return model_name_; }

    virtual geometry_msgs::PoseStamped generate_next() = 0; // Return the next pose (position)
    virtual ScriptedMotion * instanciate() = 0; // Return a newly allocated instance of the class
    virtual void update_parameters() = 0; // Called whenever set_parameters(..) is called

  protected:
    ros::Time startTime, time;
    geometry_msgs::PoseStamped translation_;
    std::string model_name_;
    XmlRpc::XmlRpcValue parameters_;
};


float s2f(std::string str);

#endif //__SCRIPTED_MOTION_H__
