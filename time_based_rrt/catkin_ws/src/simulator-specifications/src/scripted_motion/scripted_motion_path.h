#ifndef __SCRIPTED_MOTION_PATH_H__
#define __SCRIPTED_MOTION_PATH_H__

#include "scripted_motion.h"

class ScriptedMotionPath : public ScriptedMotion
{
  public:
    ScriptedMotionPath() : velocity_(1.0), height_(0.3), repeat_(false), patrol_(false), polygon_(false), is2D_(true), direction_(1)  {
      parameters_ = XmlRpc::XmlRpcValue();
      ROS_ERROR_STREAM("paramters_ init: " << parameters_);
    }
    geometry_msgs::PoseStamped generate_next();
    ScriptedMotion * instanciate() { return dynamic_cast<ScriptedMotion*>(new ScriptedMotionPath()); }

    void update_parameters();

  private:
    std::vector<geometry_msgs::Point> path_;
    int target_point_;
    geometry_msgs::Pose previous_pose_;
    double previous_sec_;
    int direction_;

    double velocity_;
    bool is2D_;      // If true, then path is specified in 2D with height_ z-value
    double height_;  // Z-value for 2D path
    bool repeat_;    // Go from start to finish and then stay still
    bool patrol_;    // If repeat: Patrol from start to finish and back - in definite
    bool polygon_;   // if repeat and not patrol: Go from start to finish, then from the finish point to the start point, then all over again
};


double inline read_int_or_float_struct_member(XmlRpc::XmlRpcValue value, std::string key, double & variable) {
  if(value.hasMember(key)) {
    if(value[key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      variable = int(value[key]);
    } else {      
      variable = double(value[key]);
    }
  }
}

double inline read_int_or_float(XmlRpc::XmlRpcValue value) {
  if(value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return int(value);
  } else {      
    return double(value);
  }
}

double inline read_bool_struct_member(XmlRpc::XmlRpcValue value, std::string key, bool & variable) {
  if(value.hasMember(key)) {
    variable = bool(value[key]);
  }
}

bool inline isEqual(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}

geometry_msgs::Point inline sub(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  geometry_msgs::Point p;
  p.x = p1.x-p2.x;
  p.y = p1.y-p2.y;
  p.z = p1.z-p2.z;
  return p;
}

geometry_msgs::Point inline add(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  geometry_msgs::Point p;
  p.x = p1.x+p2.x;
  p.y = p1.y+p2.y;
  p.z = p1.z+p2.z;
  return p;
}

double inline norm2(geometry_msgs::Point p) {
  double norm = std::abs(p.x*p.x + p.y*p.y + p.z*p.z);
  norm /= std::sqrt(norm);
  return norm;
}

geometry_msgs::Point inline scale(geometry_msgs::Point p, double s) {
  p.x *= s;
  p.y *= s;
  p.z *= s;
  return p;
}


#endif //__SCRIPTED_MOTION_PATH_H__
