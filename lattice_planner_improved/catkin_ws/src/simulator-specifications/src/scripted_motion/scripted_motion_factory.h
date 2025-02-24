#ifndef __SCRIPTED_MOTION_FACTORY_H__
#define __SCRIPTED_MOTION_FACTORY_H__

#include "scripted_motion.h"

#include <vector>
#include <map>

#include "scripted_motion_none.h"
#include "scripted_motion_circle.h"
#include "scripted_motion_path.h"


class ScriptedMotionFactory
{
  public:
    ScriptedMotionFactory();
    ScriptedMotion * create(std::string name);
    ~ScriptedMotionFactory();

  private:
    std::map<std::string,ScriptedMotion *> motions;
    std::vector<ScriptedMotion *> memory;
};

#endif //__SCRIPTED_MOTION_FACTORY_H__
