#include "Search.h"


/* Example */
#include "MemoryPoolSimple.h"
#include "ClosedSet.h"
#include "MotionPrimitive.h"

template <class State>
class StateHeuristic {
/* Default heuristic */
public:
  inline double operator()(SearchState<State>  * s, SearchState<State>  * goal) {
    return 0;
  }
};

template <class State, class Heuristic>
class AStarExplorer {
  public:
    void set_goal(SearchState<State> * goal) { this->goal = goal; }
    void set_minimum_plan_time(double minimum_plan_time) { this->minimum_plan_time = minimum_plan_time; }
    std::vector<SearchState<State> > expand(SearchState<State> * state) {
      std::vector<SearchState<State> > states;
      
      // state->action == 0 in the root. Make sure to account for this.
      
      //for(MotionPrimitive & action : possibleActions(state)) {
      MotionPrimitive action;
      {
        SearchState<State> child = *state;
        //child.action = action.id;
        child.action = 0;
        if(*state == *goal) {
          child.cost_g = state->cost_g; // The cost of standing still at the goal is 0
          child.cost_f = child.cost_g;
        }
        else {
          child.cost_g = state->cost_g + action.cost;
          child.cost_f = child.cost_g + heuristic_fn(&child, goal);
        }
        child.time = state->time + action.duration;
        child.wait_time = state->wait_time + action.wait_time;
        child.parent = state;
        //action.apply(child);  // Update child with the result of the action
        
        /*
        double proximity_cost = evaluate_proximity_cost(&state);
        if(isinf(proximity_cost)) {  // Hard constraint on collision
          continue;
        }
        child.cost_g += proximity_cost;
        child.cost_f += proximity_cost;
        */
        
        states.push_back(child);
      }
      
      return states;
    }
    bool is_goal(SearchState<State> * state, SearchState<State> * goal) {
      return *state == *goal && state->time >= minimum_plan_time;
    }
    SearchState<State> * goal;
    double minimum_plan_time;
    Heuristic heuristic_fn;
};

template <class State>
class StateOperatorLessThan {
/* Default operator */
public:
  inline bool operator()(SearchState<State>  * s1, SearchState<State>  * s2) {
    return s1->cost_f < s2->cost_f;
  }
};

template <class State, class LessThan>
class OpenSet_MultiSet {
/* Multi-set open set class. */
  typedef typename std::multiset<SearchState<State>*, LessThan> Container;
  public:
    void put(SearchState<State> * state) {
      openSet.insert(state);
    }
    SearchState<State>* pop() {
      typename Container::iterator state = openSet.begin();
      SearchState<State> * bestExploreState = *state;
      openSet.erase(state);
      return bestExploreState;
    }
    bool empty() {
      return openSet.empty();
    }    
    void clear() {
      openSet.clear();
    }    
    Container openSet;
};

template <class State>
class StateOrderingOperatorLessThan {
/* Default operator */
public:
  inline bool operator()(SearchState<State>  * s1, SearchState<State>  * s2) {
    return s1->state < s2->state;
  }
};

template <class State, class LessThan>
class ClosedSet_Set {
  typedef typename std::set<SearchState<State>*,LessThan> Container;
  public:
    bool is_visited(SearchState<State> * state) {
      return closed_set.find(state) != closed_set.end();
    }
    void put(SearchState<State> * state) {
      closed_set.insert(state);
    }
    void clear() {
      closed_set.clear();
    }
  private:
    Container closed_set;
};

class MyInterrupt {
/* Stop search after planning_time seconds. ROS-based. */
  public:
    MyInterrupt(double planning_time) : planning_time(planning_time) {}
    void reset() {
      startTime = ros::Time::now();
    }
    bool is_ok() {
      return (ros::Time::now() - startTime).toSec() < planning_time;
    }
    ros::Time startTime;
    double planning_time;
};


inline void test() {
  typedef int                                                         State;
  typedef MemoryPoolSimple<SearchState<State> >                       MyAllocator;
  typedef OpenSet_MultiSet<State, StateOperatorLessThan<State> >      MyOpenSet;
  typedef ClosedSet_Set<State, StateOrderingOperatorLessThan<State> > MyClosedSet;
  typedef AStarExplorer<State, StateHeuristic<State> >                MyExplorer;
  MyExplorer myExplorer;
  MyOpenSet myOpenSet;
  MyClosedSet myClosedSet;
  MyAllocator myAllocator;
  MyInterrupt myInterrupt(1.0);
  SearchAlgorithm<State> * searcher;
  Searcher<State, MyExplorer, MyOpenSet, MyClosedSet, MyAllocator, MyInterrupt> mySearcher(myExplorer, myOpenSet, myClosedSet, myAllocator, myInterrupt);
  searcher = &mySearcher;
  
  std::vector<SearchState<State> > result = searcher->search(State(0), State(1), 0.0);
}

