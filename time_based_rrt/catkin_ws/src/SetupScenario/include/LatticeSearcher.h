/* Author: Mattias Tiger (2018)
*/
#pragma once
#ifndef __SEARCH_H__
#define __SEARCH_H__

/* TODO: This is a work in progress.. 
         It is supposed to replace primary_search and secondary_search in latticePlanner.cpp.*/


#include "Search/Search.h"

#include "UAV/State.h"
using TrajectoryState = UAV::TrajectoryState;

/* State
  =============== */
using SearchState = search::SearchState<UAV::SearchState>;

/* Explorer
  =============== */
template <class State, class Heuristic>
class Explorer {
  public:
    Explorer() : minimum_plan_time(0.0) {};
    Explorer(double minimum_plan_time,
                  PhysicalEnvironment environment,
                  std::vector<std::vector<int> > neighbors)
      : minimum_plan_time(minimum_plan_time),
        environment(environment),
        neighbors(neighbors) {};
  
    std::vector<SearchState> expand(SearchState * state) {
      std::vector<SearchState> states;      
      for(int n = 0; n < neighbors[state->actionID].size(); n++) {
        int ID = neighbors[state->actionID][n];
        SearchState newState = SearchState(*state);
        environment.get_primitive(ID).transform_state(newState.state);
        newState.actionID = ID;
        newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
        newState.cost_f = newState.cost_g + heuristic_fn(&newState, &this->goalState);
        newState.parent = state;
        newState.time += environment.get_primitive(ID).duration;
        newState.wait_time += environment.get_primitive(ID).wait_time;
        
        double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState);
        if(isinf(proximity_cost)) {  // Hard constraint on collision
          continue;
        }
        newState.cost_g += proximity_cost;
        newState.cost_f += proximity_cost;
        
        states.push_back(newState);
      }      
      return states;
    }
    
    bool is_goal(SearchState * state, SearchState * goal) {
      return state->state == goal->state && state->time >= minimum_plan_time;
    }
    
    double minimum_plan_time;
    Heuristic heuristic_fn;
    PhysicalEnvironment environment;
    std::vector<std::vector<int> > neighbors
};
  
/* OpenSet
  =============== */
class StateOperatorLessThan {
  public:
    bool operator()(SearchState * s1, SearchState * s2) {
      return s1->cost_f < s2->cost_f;
    }
};
using OpenSet = std::multiset<SearchState*, StateOperatorLessThan>;
  
/* ClosedSet
  =============== */
  
/* Allocator
  =============== */
using Allocator = MemoryPoolSimple<SearchState>;
  
/* Interrupt
  =============== */
class Interrupt {
  /* Stop search after planning_time seconds. ROS-based. */
  public:
    Interrupt(e) : planning_time(1.0) {}
    Interrupt(double planning_time) : planning_time(planning_time) {}
    void reset() {
      startTime = ros::Time::now();
    }
    bool is_ok() {
      return (ros::Time::now() - startTime).toSec() < planning_time;
    }
    ros::Time startTime;
    double planning_time;
};

#endif
