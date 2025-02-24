#pragma once
#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#include <map>

#include "Hitbox.h"
#include "StateSpaceModel.h"

//TODO: Remove?
//#include "MotionPrimitive.h"
#include "UAV/MotionPrimitive.h"
using MotionPrimitive = UAV::MotionPrimitive;

//TODO: Remove?
//#include "State.h"
#include "UAV/State.h"
#include "Search/Search.h"
//#include "TrajectoryState.h"
using TrajectoryState = UAV::TrajectoryState;
using State = search::SearchState<UAV::SearchState>;
using SearchState = UAV::SearchState;

using namespace std;

class Obstacle {

private:

  string name;
  Hitbox obstacle_hitbox;

  //bool dynamic;
  std::vector<Hitbox> predictions;
  std::map<double,Hitbox> predictions_map;

public: // For scenarios
  bool is_advanced;
  std::string type;
  double max_speed;
  std::vector<TrajectoryState> plan;
  int plan_index;
  double previous_time;
  AABB limit;
  bool predictable = true;
  
  float soft_constraint_radius;
  
  bool display_solid = false;

public:
  StateSpaceModel * state;  // Deterministic and static if state == 0 (i.e. dynamic == false)
  double dt;  // Physical system simulation step. Resolution for predictions. Works best if it matches the motion primitive trajectory step.

  Obstacle() {
    this->obstacle_hitbox = Hitbox(0,0,0,0);
    this->name = "007";
    state = 0;
    dt = 0.5;
    is_advanced = false;
    type = "default";
    max_speed = 0.0;
    plan_index = 0;
    previous_time = -1;
    soft_constraint_radius = 1.2;
  }

  Obstacle(double x, double y, double z, double r, string name, StateSpaceModel * state = 0, double dt = 1.0/10.0) {
    this->obstacle_hitbox = Hitbox(x,y,z,r);  
    this->name = name; 
    this->state = state;
    this->dt = dt;
    is_advanced = false;
    type = "default";
    max_speed = 0.0;
    plan_index = 0;
    previous_time = -1;
    soft_constraint_radius = 1.2;
  }

  void move(double x, double y, double z) { obstacle_hitbox.set_position(x,y,z); }
  void resize(double r) { obstacle_hitbox.set_radius(r); }

  friend ostream& operator<<(ostream& os,const  Obstacle & o) {  
    os << "Obstacle name: " << o.name << "\n";
    os << o.obstacle_hitbox << "\n";
    return os;  
  }

  Hitbox & get_hitbox();

  std::string get_name() { return name; }  
  
  bool collision_with_state(const State & state, double uav_safety_radius);
  
  bool collision_with_trajectory(const std::vector<TrajectoryState> & traj, double uav_safety_radius);  
 
  /* time_n := # of time units past planning time. Planning time is at time unit 0.
     time   := actual time past planning time. Planning time is at time 0.0.
   */

  double proximity_cost(Hitbox & uav, double time);
  double proximity_cost(std::vector<TrajectoryState> & traj, Hitbox & uav, double start_time = 0);
  // Half square-exp. Play with function at: https://m.wolframalpha.com/input/?i=plot+3*exp%28-5*x*x%29%29%2C+x%3D0..5%2C+y%3D-1..5
  // NOTE: This is per TRAJ POINT, at 10Hz.
  double cost_func(double dist) { return dist > 0 ? 0.1*exp(-2*dist*dist) : std::numeric_limits<float>::infinity(); }  // Hard constraint on collision, cost on proximity.

  // Static collision check
  bool in_collision(Hitbox & uav);
  bool in_collision_static(std::vector<TrajectoryState> & traj, Hitbox & uav, double start_time = 0);
  
  // Dynamic collision check
  bool in_collision(Hitbox & uav, double time);
  bool in_collision(std::vector<TrajectoryState> & traj, Hitbox & uav, double start_time = 0);
  bool in_collision(Hitbox & uav, double time, double duration);  // For a time frame
  Hitbox & predict(double time, bool conservative);
  Hitbox predict(double time, double time_duration);
  void clear_predictions(); 
 
};


#endif
