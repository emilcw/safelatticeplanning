#ifndef __COMMON_SIMULATOR_H__
#define __COMMON_SIMULATOR_H__

#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <sstream>


// Useful templates
  template<class TYPE>
  bool searchGetParam_(std::string name, TYPE & param) {
    std::string path;
    bool isResolved = ros::param::search(name, path);
    if(isResolved) {
      ros::param::get(path, param);
    }
    return isResolved;
  }

  // Without default value - parameter must exist!
  template<class TYPE>
  bool searchGetParam(std::string name, TYPE & param) {
    if(searchGetParam_(name, param) == false) {
      ROS_ERROR_STREAM("Param \"" << name << "\" could not be resolved.");
      return false;
    }
    return true;
  }

  // With default value - parameter need not be set!
  template<class TYPE>
  bool searchGetParam(std::string name, TYPE & param, TYPE defaultValue) {
    if(searchGetParam_(name, param) == false) {
      ROS_WARN_STREAM("Param \"" << name << "\" could not be resolved. It is set to default value");
      param = defaultValue;
      return false;
    }
    return true;
  }


#endif // __COMMON_SIMULATOR_H__
