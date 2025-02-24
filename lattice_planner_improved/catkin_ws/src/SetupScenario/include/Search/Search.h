/* Author: Mattias Tiger (2018)
*/
#pragma once
#ifndef __SEARCH_H__
#define __SEARCH_H__

#include <vector>

#include "SearchState.h"

namespace search {

template <class State>
class SearchAlgorithm {
  public:
    virtual ~SearchAlgorithm() {};
    virtual SearchState<State> * search_implementation(SearchState<State> start, 
                                                       SearchState<State> goal) = 0;
    std::vector<SearchState<State> > search(State start, 
                                            State goal,
                                            double start_time) {
      SearchState<State> start_(start, start_time);
      SearchState<State> goal_(goal);
      SearchState<State> * goal__ = search_implementation(start_, goal_);
      if(goal__ == 0)
        return std::vector<SearchState<State> >();
      return construct_plan(*goal__);                                

    }

  protected:
    std::vector<SearchState<State> > construct_plan(SearchState<State> goal) {
      std::vector<SearchState<State> > plan;
      SearchState<State> * state = &goal;
      plan.push_back(*state);
      while(!state->is_root()) {
        state = state->parent;
        plan.push_back(goal);
      }
      return plan;
    }
};



template <class State, class Explorer, class OpenSet, class ClosedSet, class Allocator, class Interrupt>
class Searcher : public SearchAlgorithm<State> {
  public:
    Searcher(Explorer & explorer,
             OpenSet & openSet,
             ClosedSet & closedSet,
             Allocator & allocator,
             Interrupt & interrupt)
             : allocator(allocator), openSet(openSet),
               closedSet(closedSet), explorer(explorer),
               interrupt(interrupt) {};
    ~Searcher() {}
    SearchState<State> * search_implementation(SearchState<State> start, SearchState<State> goal) override {
      SearchState<State> * rootState = allocator.allocate();
      *rootState = start;
      openSet.put(rootState);
      interrupt.reset();
      
      while(interrupt.is_ok() && !openSet.empty()) {
        SearchState<State> * state = openSet.pop();
        
        if(explorer.is_goal(state, &goal)) {
          return state;
        }
          
        if(!closedSet.is_visited(state)) {
          closedSet.put(state); 
               
          std::vector<SearchState<State> > neighbors = explorer.expand(state);
          for(SearchState<State> & neighbor : neighbors) {
              SearchState<State> * newState = allocator.allocate();
              *newState = neighbor;
              openSet.put(newState);           
          }
        }
      }
      return 0; // Search failed
    }
    
  private:
    Allocator & allocator;
    OpenSet & openSet;
    ClosedSet & closedSet;
    Explorer & explorer;
    Interrupt & interrupt;  
};

}

#endif
