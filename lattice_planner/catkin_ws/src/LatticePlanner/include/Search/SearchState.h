/* Author: Mattias Tiger (2018)
*/
#pragma once
#ifndef __SEARCH_SEARCH_STATE_H__
#define __SEARCH_SEARCH_STATE_H__

#include <iostream>

namespace search {

template <class State>
class SearchState {
  public:
    State state;
    SearchState * parent = 0;
    int actionID = -1;
    double time = 0.0;
    int wait_time = 0;

    /*
    Cost from start state to current state in the std::vector<State> plan.
    
    Example:
    If we have plan: state1 -> state2 -> state3
    Then state3.cost_g will be equal to the distance when travelling from 
    state1 to state3.
    */
    double cost_g = 0;

    /*
    Cost from start_state to current state (cost_g) + heuristic estimate to goal
    NOTE: Check the which heuristic_fkn() is used in LatticePlanner.cpp and LatticePlanner.h
    since one of them takes the distance * 0.5 and one takes distance * 5.0 (weird right
    but apparently it improves performance)

    Example:
    Plan: state1 -> state2 -> state3 -> state4 -> state5
    Then state3.cost_f will be equal to the distance when travlling from state1
    to state3 (i.e state3.cost_g) + heuristic_fkn(state3, state5)
    */  
    double cost_f = 0; 

  public:
	  SearchState() {}
	  SearchState(State state) : state(state) {}
	  SearchState(State state, double time) : state(state), time(time) {}
	
	  SearchState(const SearchState& other) {
	    this->parent      = other.parent;
	    this->time        = other.time;
      this->wait_time   = other.wait_time;
	    this->actionID    = other.actionID;
	    this->cost_f      = other.cost_f;
	    this->cost_g      = other.cost_g;
      this->state       = other.state;
	  }

	  void operator=(const SearchState& other) {
	    this->time        = other.time;
	    this->wait_time   = other.wait_time;
	    this->parent      = other.parent; 
	    this->actionID    = other.actionID;
	    this->cost_f      = other.cost_f;
	    this->cost_g      = other.cost_g;
	    this->state       = other.state;	
	  }

    bool operator==(const SearchState& other) {
      return this->state  == other.state;
    }
    
    bool is_root() {
      return parent == 0;
    }
 

    friend std::ostream& operator<<(std::ostream& os, SearchState &o) {  
      if(o.wait_time != 0)
        os << "$: " << o.cost_g << ", t: " << o.time << " wt: " << o.wait_time << " " << o.state << " | Action=" << o.actionID;  
      else
        os << "$: " << o.cost_g << ", t: " << o.time << " " << o.state << " | Action=" << o.actionID;  
      return os;  
    }
};

}

template <class State>
inline bool operator<(const search::SearchState<State> & s1, const search::SearchState<State> & s2) {
  return s1.wait_time < s2.wait_time || 
         (s1.wait_time == s2.wait_time && s1.state < s2.state);
}

template <class State>
inline bool operator==(const search::SearchState<State> & s1, const search::SearchState<State> & s2) {
  return s1.wait_time == s2.wait_time && s1.state == s2.state;
}
         

#endif
