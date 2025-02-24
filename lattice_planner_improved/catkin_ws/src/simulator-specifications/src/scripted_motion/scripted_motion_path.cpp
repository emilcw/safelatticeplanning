#include "scripted_motion_path.h"

geometry_msgs::PoseStamped ScriptedMotionPath::generate_next() {
  double sec = ros::Time::now().toSec();
  if(previous_sec_ == 0) {  // For some reason this is necessary..
    previous_sec_ = sec;
  }
  double dt = sec-previous_sec_;
  previous_sec_ = sec;

  geometry_msgs::Pose translation = translation_.pose;
  geometry_msgs::Pose pose;

  bool finished = false;
  if(isEqual(path_[target_point_],previous_pose_.position)) {
    if(target_point_ == path_.size()-1) {
      if(polygon_) {
        target_point_ = 0;
      } else {
        if(patrol_) {
          direction_ *= -1;
          target_point_ += direction_;
        } else {
          if(repeat_) {
            previous_pose_.position = path_[0];
            target_point_ = 1;
          } else {
            finished = true;
          }
        }
      }
    } else {
      if(target_point_ == 0 && direction_ < 0) {
        direction_ *= -1;
      }
      target_point_ += direction_;
    }
  }

  if(!finished) {
    double step = norm2(sub(path_[target_point_],previous_pose_.position));
    if(step < velocity_*dt) {
      previous_pose_.position = path_[target_point_];
    } else {
      previous_pose_.position = add(previous_pose_.position, scale( scale( sub(path_[target_point_],previous_pose_.position), 
                                                                           1.0/step), 
                                                                    velocity_*dt));
    }
  }
  
  
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose = previous_pose_;
  return poseStamped;
}


void ScriptedMotionPath::update_parameters() {
  read_int_or_float_struct_member(parameters_, "vel", velocity_);
  read_bool_struct_member(parameters_, "repeat", repeat_);
  read_bool_struct_member(parameters_, "patrol", patrol_);
  read_bool_struct_member(parameters_, "polygon", polygon_);
  read_bool_struct_member(parameters_, "is2D", is2D_);
  read_int_or_float_struct_member(parameters_, "z", height_);
  
  //ROS_ERROR_STREAM("repeat: " << repeat_ << ", patrol: " << patrol_ << ", polygon: " << polygon_ << ", is2D: " << is2D_);
  XmlRpc::XmlRpcValue path = parameters_["path"];
  for(int n = 0; n < path.size(); n++) {
    geometry_msgs::Point point;
    point.x = read_int_or_float(path[n][0]);
    point.y = read_int_or_float(path[n][1]);
    if(is2D_) {
      point.z = height_;
    } else {
      point.z = read_int_or_float(path[n][2]);
    }
    path_.push_back(point);
  } 

  target_point_ = 1;
  previous_pose_.position = path_[0];
  previous_sec_ = ros::Time::now().toSec();

}
