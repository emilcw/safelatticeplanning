#pragma once
#ifndef __UAV_MOTION_PRIMITIVE_H__
#define __UAV_MOTION_PRIMITIVE_H__

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <string>
#include <vector>

#include "../Hitbox.h"

#include "UAV/State.h"


using namespace std;

//#define debug_enable
#ifndef debug_enable
  #define DLOG(stream) 
#else
  #define DLOG(stream) cerr << stream
#endif

namespace UAV {
  /* Floating point type */
  using FT = float;
  
  /* Motion primitive class */
  class MotionPrimitive {
    public:
      enum PrimitiveType {
        PRIMARY = 0,
        SECONDARY = 1
      };
    
    public:
      int ID;                               // Unique idenfier
      int group;                            // Motion primitive group
      string name;
      vector<string> map_label;   //"to-from-state-label" (concerns this->to and this->from)
      vector<string> trajectory_state_label; // (concerns this->trajectory)
      PrimitiveType type;
      FT cost;
      FT duration;
      FT wait_time;                         // Used for "standing still"-actions
      UAV::SearchState from;                     // Prerequisite for the action
      UAV::SearchState to;                       // Effect of the action
      vector<TrajectoryState> trajectory;  // Realization of the action
      
      Hitbox hitbox;                        // A conservatice bound containing the UAV (and and its safety margins) for the full duration of the motion primitive
    
    public:
      MotionPrimitive()
        : ID(-1), group(-1), cost(100000), duration(-1), wait_time(-1), type(PrimitiveType::PRIMARY) {}
      MotionPrimitive(int ID, int group, string name, double duration, double cost, 
                      vector<TrajectoryState> trajectory, int wait_time = 0)
        : ID(ID), group(group), name(name), duration(duration), cost(cost), 
          trajectory(trajectory), wait_time(wait_time), type(PrimitiveType::PRIMARY) {}
    
      bool load_from_file(string primitive_file_path, int startID);
      
      void transform_state(UAV::SearchState & state);
      vector<TrajectoryState> apply(const SearchState & state, double time);
      
      Hitbox calculate_hitbox(Hitbox vehicle);
      
      Hitbox get_hitbox(UAV::SearchState & state);  // Translate hitbox and return it
      
      int trajectory_length() { return trajectory.size(); }
  
      void clean_up_numbers();
  
      friend ostream& operator<<(ostream& os, MotionPrimitive & o) {  
        os << "ID: " << o.ID << ", Group: " << o.group << ", Name: " << o.name << ", Cost: " 
           << o.cost << ", Duration: " << o.duration << " seconds, " 
           << "Wait-Time: " << o.wait_time << " \n";
        for(int n = 0; n < o.trajectory.size(); n++) {
          os << o.trajectory[n] << "\n";
        }
        return os;
      }
  };

  /* Utilities */
  bool has_prefix(string prefix, string string);  

}

#endif
