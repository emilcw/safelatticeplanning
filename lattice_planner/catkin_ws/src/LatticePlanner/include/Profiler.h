#pragma once
#ifndef __PROFILER_H__
#define __PROFILER_H__

#include "ros/ros.h"
#include <sstream>

class Profiler {
public:
  struct TimeMark {
    TimeMark(std::string name, double seconds) : name(name), seconds(seconds) {}
    std::string name;
    double seconds;
  };

public:
  Profiler() {
    marks.reserve(100);
    reset();
  }
  void reset() {
    is_stopped_ = false;
    marks.clear();
    start_time_ = ros::Time::now();
    previous_time_ = start_time_;
    stop_time_ = start_time_;
  }
  void stop() {
    stop_time_ = ros::Time::now();
    is_stopped_ = true;
  }
  void mark(std::string name) {
    marks.push_back(TimeMark(name, (ros::Time::now() - previous_time_).toSec()));
    previous_time_ = ros::Time::now();
  }

  std::string to_string() {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }
  	
  friend std::ostream& operator<<(std::ostream& os, Profiler &o) {
      if(!o.is_stopped_) {
        o.stop_time_ = ros::Time::now();
      }
      for(int n = 0; n < o.marks.size(); n++) {
        os << o.marks[n].name << ": " << std::setprecision(6) << o.marks[n].seconds << " seconds\n";
      }
      os << "-----------------\n";
      os << "Total: " <<  std::setprecision(6) << (o.stop_time_ - o.start_time_).toSec() << " seconds\n";
      return os;  
  }
  
  double total() { return (stop_time_ - start_time_).toSec(); }
  
  std::vector<TimeMark> get_marks() {return marks;}
  
  std::map<std::string,double> get_marks_map() {
    std::map<std::string,double> map;
    for(int n = 0; n < marks.size(); n++) {
      map[marks[n].name] = marks[n].seconds;
    }
    return map;
  }

private:
  ros::Time start_time_;
  ros::Time previous_time_;
  ros::Time stop_time_;
  bool is_stopped_;
  std::vector<TimeMark> marks;

};

#endif // __PROFILER_H__
