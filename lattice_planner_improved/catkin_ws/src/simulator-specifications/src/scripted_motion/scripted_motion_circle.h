#ifndef __SCRIPTED_MOTION_CIRCLE_H__
#define __SCRIPTED_MOTION_CIRCLE_H__

#include "scripted_motion.h"

class ScriptedMotionCircle : public ScriptedMotion
{
  public:
    ScriptedMotionCircle() : radius_(1.0) {}
    geometry_msgs::PoseStamped generate_next();
    ScriptedMotion * instanciate() { return dynamic_cast<ScriptedMotion*>(new ScriptedMotionCircle()); }

    void update_parameters();

  private:
    float radius_;
};

#endif //__SCRIPTED_MOTION_CIRCLE_H__
