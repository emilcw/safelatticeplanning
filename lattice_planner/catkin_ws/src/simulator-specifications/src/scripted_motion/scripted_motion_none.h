#ifndef __SCRIPTED_MOTION_NONE_H__
#define __SCRIPTED_MOTION_NONE_H__

#include "scripted_motion.h"

class ScriptedMotionNone : public ScriptedMotion
{
  public:
    geometry_msgs::PoseStamped generate_next() { 
      geometry_msgs::PoseStamped next = translation_;
      next.header.stamp = ros::Time::now();
      return next;
    }
    ScriptedMotion * instanciate() { return dynamic_cast<ScriptedMotion*>(new ScriptedMotionNone()); }
    void update_parameters() {}
};

#endif //__SCRIPTED_MOTION_NONE_H__
