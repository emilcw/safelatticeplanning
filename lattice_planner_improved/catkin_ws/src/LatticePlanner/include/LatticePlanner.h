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

#include "UAV/State.h"
#include "Search/Search.h"
#include "Obstacle.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


using TrajectoryState = UAV::TrajectoryState;
using State = search::SearchState<UAV::SearchState>;
using SearchState = UAV::SearchState;
using namespace std;


namespace LP 
{
  enum TAG {INFO, WARN, ERROR};
  enum SEARCH_TYPE {EXPAND, PRIMARY, SECONDARY};
}

#define LP_DEBUG(tag,stream) {std::stringstream ss; ss << stream; this->debug_log(tag,ss.str());}


class FrontierLookupLessThan 
{
  public:
    bool operator()(const search::SearchState<State> & s1, const search::SearchState<State> & s2) 
    {
      return s1 < s2;
    }
};


class StateOperatorLessThan 
{
  public:
    bool operator()(State * s1, State * s2) 
    {
      return s1->cost_f < s2->cost_f;
    }
};


struct ClosestToGoalLessThan 
{
  ClosestToGoalLessThan() {}
  ClosestToGoalLessThan(SearchState goal, bool use_stand_still) : goal(goal), use_stand_still(use_stand_still) {}

  bool operator()(State * new_state, State * best) 
  {

    double speed1 = new_state->state.velocity.squaredNorm();
    double speed2 = best->state.velocity.squaredNorm();

    // If new_state speed is too large, keep best_state
    // If best_state speed is too large, switch to new_state
    // Prefer this if we can use stand_still, otherwise hard to use.
    if(use_stand_still)
    {
      if(speed1 > 0.01) return false;
      if(speed2 > 0.01) return true;
    }
    
    //If not stand-still is available, pick closest one or the one with lowest cost
    double distance1 = (new_state->state.position - goal.position).norm();
    double distance2 = (best->state.position - goal.position).norm();

    return distance1 < distance2 || (distance1 == distance2 && new_state->cost_f <= best->cost_f);
  }

  double distance(State * s) 
  {
    double speed = s->state.velocity.squaredNorm();
    if(speed > 0.01) return 1e9;
    double distance = (s->state.position - goal.position).norm();
    return distance;
  }

  SearchState goal;
  bool use_stand_still;
};


class LatticePlanner {

  typedef std::multiset<State*, StateOperatorLessThan> FrontierContainer;

  public:

    // -- Constructor and destructor --
    LatticePlanner() {}
    ~LatticePlanner() {}

    // -- Variables --
    bool use_stand_still;
    bool use_only_geometric_search;
    bool use_geometric_secondary_search;
    bool visualize_frontiers;
    bool emergency_trajectories_ = false;
    bool survival_planning_ = false;
    bool best_effort; // Currently not used, replaced with survial planning

    double uav_safety_radius;
    double plan_duration_minimum;

    Hitbox uav_hitbox;

    State rootState;
    State goalState;     //True goal state, can never be changed 
    State tempGoalState; //Temporary goal state, used when we cannot find a path to the true goal state
    
    PhysicalEnvironment environment;

    std::string closed_set_type;

    // -- Main functions --
    bool do_planning_cycle(SearchState fromSearchState, 
                          SearchState goalState, 
                          double planning_time, 
                          std::vector<TrajectoryState> &planned_trajectory, 
                          double start_time, 
                          TrajectoryState collision_state,
                          double time_until_next_collision, 
                          bool current_plan_valid = false);


    bool expand_frontier_from_state(State * root_state,
                                    double start_time,
                                    FrontierContainer & frontier,
                                    ClosedSet * closed_set);

    bool expand_emergency_from_state(SearchState from, 
                                    SearchState goal, 
                                    double planning_time, 
                                    vector<TrajectoryState> &planned_trajectory, 
                                    double start_time,
                                    TrajectoryState collision_state,
                                    double time_until_next_collision,
                                    bool current_plan_valid); 


    // -- Planner helper functions --
    void update_octomap(octomap::OcTree* octree);
    void set_solution_path(std::vector<State*> v) { solution_path = v; }
    void set_allowed_volume(AABB allowed_volume) { environment.set_allowed_volume(allowed_volume); }
    void set_allowed_volume(double minX, double maxX,
                            double minY, double maxY,
                            double minZ, double maxZ){ environment.set_allowed_volume(minX, maxX, minY, maxY, minZ, maxZ); }
    void print_obstacles() { environment.print_obstacles(); }
    void add_obstacle(Obstacle obstacle) { environment.add_obstacle(obstacle); }
    void clear_obstacles() {environment.clear_obstacles(); }
    void get_longest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory);
    void get_closest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory);
    void debug_log(LP::TAG tag, std::string log);
    void generate_solution_trajectory(std::vector<TrajectoryState> & trajectory);

    bool has_plan();
    bool best_effort_enabled() { return this->best_effort; }
    bool isGoalInCollision();
    bool isStartInCollision();
    bool frontier_empty();
    bool is_root_state_in_collision();
    bool is_state_initalized(TrajectoryState state)
    {
    return state.position.x() != std::numeric_limits<float>::infinity() && 
           state.position.y() != std::numeric_limits<float>::infinity() &&
           state.position.z() != std::numeric_limits<float>::infinity();
    }
    
    bool initialize(std::string path, 
                    std::vector<int> primitive_amount_in_group, 
                    double uav_safety_radius, 
                    Profiler * profiler,
                    ros::NodeHandle nh,
                    bool use_stand_still = true,
                    bool use_geometric_secondary_search = true,
                    bool use_only_geometric_search = false,
                    double plan_duration_minimum = 5.0,
                    std::string closed_set_type = "SetWaitTime",
                    bool best_effort = true,
                    bool write_debug_to_console = false,
                    bool visualize_frontiers = false,
                    bool emergency_trajectories = false,
                    bool survival_planning = false);

    double get_plan_duration_minimum() { return plan_duration_minimum; }

    State * find_best_destination(FrontierContainer & frontier, double min_time, SearchState goal);
    State create_new_state(State previous_state, int primitiveID);

    MotionPrimitive & get_primitive(int index) { return environment.get_primitive(index); }

    FrontierContainer & get_frontier() {return frontier;}
    ClosedSet * get_closed_set() {return exploredStates;}

    std::vector<State*>  get_solution_path() { return solution_path; }
    std::vector<State> clone_solution_path();
    std::vector<Obstacle>  get_obstacles() { return environment.get_obstacles(); }
    std::vector<Obstacle> & get_static_obstacles() { return environment.get_static_obstacles(); }
    std::vector<Obstacle> & get_dynamic_obstacles() { return environment.get_dynamic_obstacles(); }
    std::vector<MotionPrimitive> & get_primitives() { return environment.get_primitives(); }
    

  private:

    // -- Variables --
    bool write_debug_to_console;
    
    ros::Publisher expand_viz_pub_, primary_viz_pub_, secondary_viz_pub_, solution_path_viz_pub_; 
    
    Profiler * profiler;
            
    MemoryPoolSimple<State> memoryPool;

    State * longest_collision_free_goal;
    State * closest_collision_free_goal;

    FrontierContainer frontier;
    FrontierContainer secondary_frontier;
    ClosedSet * exploredStates;
    ClosestToGoalLessThan closestToGoalLessThan;

    std::vector<State*> solution_path;
    std::map<State,FrontierContainer::iterator,FrontierLookupLessThan> frontier_lookup;

    // -- Main functions --
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

    // -- Planner helper functions --
    void set_visited(State * state);
    void generate_solution_path(State * state);
    void add_to_frontier(FrontierContainer & frontier, State * state);
    void print_frontier();
    void print_explored_states();
    void print_trajectory(std::vector<TrajectoryState> & trajectory);
    void add_to_frontier(State * state);

    bool goal_found();
    bool is_goal(State * state);
    bool possible_to_expand(State & state, int id);
    bool is_visited(State & state);
    bool make_unique_in_frontier(FrontierContainer & frontier, State & state);
    bool is_frontier_empty(FrontierContainer & frontier);
    
    State * pop_from_frontier();
    State * pop_from_frontier(FrontierContainer & frontier);

    // -- Visualization --
    void publish_visualization_frontier(FrontierContainer & frontier, LP::SEARCH_TYPE type);
    void publish_visualization_solution_path(std::vector<State*> solution_path);


  template<class T>
  std::string to_string(T value) 
  {
    std::stringstream ss;
    ss << value;
    return ss.str();
  }
};




/*
Heuristic function to estimate distance to goal. This is used in 
LatticePlanner::expand_frontier_from_root and primary_search.
Variable greed can be tuned to control the planning process.
It should be 1.0 but planner gets slow without greed.
*/
inline double heuristic_fn(State * s, State * goal) 
{
  double greed = 0.5;
  return greed * (s->state.position - goal->state.position).norm();
}


/*
Heuristic function to estimate distance to goal. This is used in 
LatticePlanner::secondary_search. Variable greed can be tuned to 
control the planning process.
It should be 1.0 but planner gets slow without greed.
*/
inline double heuristic_fn2(State * s, State * goal) 
{
  double greed = 5.0;
  return greed * (s->state.position - goal->state.position).norm();
}


bool is_plan_leading_to_goal(std::vector<State> & plan, State & goal);
bool is_plan_in_the_past(std::vector<State> & plan);


#endif // __LATTICE_PLANNER_H__
