#include "scripted_motion_factory.h"

ScriptedMotionFactory::ScriptedMotionFactory() {
  motions["none"] = new ScriptedMotionNone();
  motions["circle"] = new ScriptedMotionCircle();
  motions["path"] = new ScriptedMotionPath();
  // Add additional motion types here
}

ScriptedMotion * ScriptedMotionFactory::create(std::string motion_name) {
  if(motions.find(motion_name) == motions.end()) {
    ROS_ERROR_STREAM("No motion with name '" << motion_name << "' in motion factory");
    return 0;
  }
  ScriptedMotion * motion = motions[motion_name]->instanciate();
  memory.push_back(motion);
  return motion;
}

ScriptedMotionFactory::~ScriptedMotionFactory() {
  for(std::vector<ScriptedMotion*>::iterator it = memory.begin(); it != memory.end(); it++) {
    delete *it;
  }
}

