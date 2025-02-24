/* Author: Oskar Ljungqvist and Mattias Tiger
*
*
*/
#pragma once
#ifndef __LATTICE_PLANNER_H__
#define __LATTICE_PLANNER_H__

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <assert.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <mav_msgs/common.h>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <sstream>
#include <list>
#include <vector>
#include <set>
#include <map>
#include <fstream>

#include "Hitbox.h"
#include "Profiler.h"
#include "MemoryPoolSimple.h"
#include "ClosedSet.h"
#include "PhysicalEnvironment.h"
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#undef NDEBUG
#include "AssertMsg.h"

//TODO: Remove?
//#include "State.h"
#include "UAV/State.h"
#include "Search/Search.h"
//#include "TrajectoryState.h"
using TrajectoryState = UAV::TrajectoryState;
using State = search::SearchState<UAV::SearchState>;
using SearchState = UAV::SearchState;

#include "Obstacle.h"

// Lattice Planner debug macro
namespace LP {
  enum TAG {INFO, WARN, ERROR};
}
#define LP_DEBUG(tag,stream) {std::stringstream ss; ss << stream; this->debug_log(tag,ss.str());}

class FrontierLookupLessThan {
public:
  bool operator()(const search::SearchState<State> & s1, const search::SearchState<State> & s2) {
    return s1 < s2;
  }
};


using namespace std;

/* Graph Search classes/functions ------------------------ START */
class StateOperatorLessThan {
public:
  bool operator()(State * s1, State * s2) {
    return s1->cost_f < s2->cost_f;
  }
};

inline double heuristic_fn(State * s, State * goal) {
  //should be 1.0 but the planner gets slow without a little greed
  //return 1.0*(s->state.position - goal->state.position).norm() * 0.2;  
  //return 1.0*(s->state.position - goal->state.position).norm()*5;
  //return 1.0*(s->state.position - goal->state.position).norm()*0.5;
  return 0.5*(s->state.position - goal->state.position).norm();
}
inline double heuristic_fn2(State * s, State * goal) {
  //should be 1.0 but the planner gets slow without a little greed 
  return 5.0*(s->state.position - goal->state.position).norm(); // 1.6 is fastest (and only) velocity for geometric primitives. *100 to make them less optimal when re-planning with increased dynamic horizon
}

// Not considering minimal_plan_time
struct ClosestToGoalLessThan {
  ClosestToGoalLessThan() {}
  ClosestToGoalLessThan(SearchState goal) : goal(goal) {}

  bool operator()(State * s1, State * s2) {
    double speed1 = s1->state.velocity.squaredNorm();
    double speed2 = s2->state.velocity.squaredNorm();
    if(speed1 > 0.01) return false;
    if(speed2 > 0.01) return true;

    double distance1 = (s1->state.position - goal.position).norm();
    double distance2 = (s2->state.position - goal.position).norm();
    return distance1 < distance2 || 
           (distance1 == distance2 && s1->cost_f <= s2->cost_f);
  }

  double distance(State * s) {
    double speed = s->state.velocity.squaredNorm();
    if(speed > 0.01) return 1e9;
    double distance = (s->state.position - goal.position).norm();
    return distance;
  }

  SearchState goal;
};


/* Graph Search classes/functions ------------------------ END */

class LatticePlanner {
private:
  typedef std::multiset<State*, StateOperatorLessThan> FrontierContainer;

public:
  
  LatticePlanner() {}
  ~LatticePlanner() {}

  bool initialize(std::string path, 
                  std::vector<int> primitive_amount_in_group, 
                  double uav_safety_radius, 
                  Profiler * profiler,
                  bool use_stand_still = true,
                  bool use_geometric_secondary_search = true,
                  bool use_only_geometric_search = false,
                  double plan_duration_minimum = 5.0,
                  std::string closed_set_type = "SetWaitTime",
                  bool best_effort = true,
                  bool write_debug_to_console = false);
  //TODO: Implement a public API for planning-methods
  
  std::vector<TrajectoryState> bestPlan;
  
  bool status;
  double uav_safety_radius; //Used for collision checking
  Hitbox uav_hitbox;

  bool has_plan();
  bool best_effort_enabled() { return this->best_effort; }

  std::vector<State*>  get_solution_path() { return solution_path; }
  void  set_solution_path(std::vector<State*> v) {
    solution_path = v;
  }
  std::vector<State> clone_solution_path();
  
  bool do_planning_cycle(SearchState fromSearchState, SearchState goalState, double planning_time, std::vector<TrajectoryState> &planned_trajectory, double start_time);
  bool do_planning_cycle_old(SearchState fromSearchState, SearchState goalState, double planning_time, std::vector<TrajectoryState> &planned_trajectory, double start_time);

  bool expand_frontier_from_state(State * root,
                                  double start_time,
                                  FrontierContainer & frontier,
                                  ClosedSet * closed_set);  

  void goal_reached();

  std::vector<Obstacle>  get_obstacles() { return environment.get_obstacles(); }
  std::vector<Obstacle> & get_static_obstacles() { return environment.get_static_obstacles(); }
  std::vector<Obstacle> & get_dynamic_obstacles() { return environment.get_dynamic_obstacles(); }
  std::vector<MotionPrimitive> & get_primitives() { return environment.get_primitives(); }
  MotionPrimitive & get_primitive(int index) { return environment.get_primitive(index); }

  void print_obstacles() { environment.print_obstacles(); }
  void add_obstacle(Obstacle obstacle) { environment.add_obstacle(obstacle); }
  void clear_obstacles() {environment.clear_obstacles(); }
  void set_allowed_volume(AABB allowed_volume) { environment.set_allowed_volume(allowed_volume); }
  void set_allowed_volume(double minX, double maxX,
                          double minY, double maxY,
                          double minZ, double maxZ) {
    environment.set_allowed_volume(minX, maxX, minY, maxY, minZ, maxZ); 
  }

  bool isGoalInCollision();
  bool isStartInCollision();

  double get_plan_duration_minimum() { return plan_duration_minimum; }

  void get_longest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory);
  void get_closest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory);


  /* Graph Search methods ------------------------ START */
  bool frontier_empty();
  /* Graph Search methods ------------------------ END */

  bool is_root_state_in_collision();
  
  void debug_log(LP::TAG tag, std::string log);
  void generate_solution_trajectory(std::vector<TrajectoryState> & trajectory);
  State create_new_state(State previous_state, int primitiveID);

private:

  bool write_debug_to_console;
  
  bool search_old(double planning_time, double start_time);
  bool primary_search(double planning_time, 
                      double start_time,
                      double plan_duration_minimum,
                      std::vector<std::vector<int> > & neighbors,
                      FrontierContainer & frontier,
                      ClosedSet * closed_set);
  bool secondary_search(double planning_time, 
                        double start_time,
                        std::vector<std::vector<int> > & neighbors,
                        FrontierContainer & frontier,
                        ClosedSet * closed_set);
  
  bool goal_found();

  

  
  /* Graph Search methods ------------------------ START */
  bool is_goal(State * state);
  State * pop_from_fontier();
  bool possible_to_expand(State & state, int id);
  bool is_visited(State & state);
  void add_to_frontier(State * state);
  bool make_unique_in_frontier(FrontierContainer & frontier, State & state);
  void set_visited(State * state);
  void generate_solution_path();
  /* Graph Search methods ------------------------ END */

  /* Graph Search methods (NEW)------------------------ START */
  bool is_frontier_empty(FrontierContainer & frontier);
  State * pop_from_fontier(FrontierContainer & frontier);
  void add_to_frontier(FrontierContainer & frontier, State * state);
  /* Graph Search methods ------------------------ END */
  
  template<class T>
  std::string to_string(T value) {
    std::stringstream ss;
    ss << value;
    return ss.str();
  }

  /* Debuging print outs ------------------------- START */
  void print_frontier();
  void print_explored_states();
  void print_trajectory(std::vector<TrajectoryState> & trajectory);
  /* Debuging print outs ------------------------- END */

private:

  Profiler * profiler;
  /*
  std::vector<MotionPrimitive> primitives;
  std::vector<MotionPrimitive> geometric_primitives;
  std::vector<std::vector<int> > primitive_adjacency_index; // Not a matrix.
  std::vector<std::vector<int> > geometric_primitive_adjacency_index; // Not a matrix.
  int geometric_index_start;*/
           
  MemoryPoolSimple<State> memoryPool;

  /* Graph Search properties ------------------------ START */

public:
  State rootState;
  State goalState;
  
  PhysicalEnvironment environment;

private:
  FrontierContainer frontier;
  FrontierContainer secondary_frontier;
  ClosedSet * exploredStates;
  std::map<State,FrontierContainer::iterator,FrontierLookupLessThan> frontier_lookup;
  
  std::vector<State*> solution_path;
  /* Graph Search methods ------------------------ END */
  
  State * longest_collision_free_goal;
  State * closest_collision_free_goal;
  ClosestToGoalLessThan closestToGoalLessThan;

public:
  /* Settings */
  double plan_duration_minimum;
  bool use_stand_still;
  bool use_only_geometric_search;
  bool use_geometric_secondary_search;
  std::string closed_set_type;
  bool best_effort; // If no plan to goal is found or if no collision free plan is found, do best effort plan
  void update_octomap(octomap::OcTree* octree);

public:
  FrontierContainer & get_frontier() {return frontier;}
  ClosedSet * get_closed_set() {return exploredStates;}

  State * find_best_destination(FrontierContainer & frontier, double min_time);
};

bool is_plan_leading_to_goal(std::vector<State> & plan, State & goal);
bool is_plan_in_the_past(std::vector<State> & plan);


#endif // __LATTICE_PLANNER_H__
