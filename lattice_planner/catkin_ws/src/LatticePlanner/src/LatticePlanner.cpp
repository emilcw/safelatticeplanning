#include "LatticePlanner.h"

bool LatticePlanner::initialize(std::string path, 
                  std::vector<int> primitive_amount_in_group, 
                  double uav_safety_radius, 
                  Profiler * profiler,
                  bool use_stand_still,
                  bool use_geometric_secondary_search,
                  bool use_only_geometric_search,
                  double plan_duration_minimum,
                  std::string closed_set_type,
                  bool best_effort,
                  bool write_debug_to_console) {

  memoryPool.init(50000000);

  this->profiler = profiler;
  this->write_debug_to_console = write_debug_to_console;

  int maxValue = std::numeric_limits<int>::max()/2-1;
  //environment.set_allowed_volume(-maxValue, maxValue, -maxValue, maxValue, -maxValue, maxValue);
  environment.set_allowed_volume(-maxValue, maxValue, -maxValue, maxValue, 1.5, maxValue);
  /* If flying indoors 
     x: -3.3 < x < 3.3
     y: -3.3 < y < 2.3
     z: 1.5 < z < 3.5       */
  //environment.set_allowed_volume(-3.3, 3.3, -3.3, 2.3, 1.5, 3.5);

  this->uav_safety_radius = uav_safety_radius;
  this->uav_hitbox = Hitbox(0,0,0,uav_safety_radius); 
  
  // Options
  this->best_effort = best_effort;
  this->use_only_geometric_search = use_only_geometric_search;
  this->use_geometric_secondary_search = !use_only_geometric_search && use_geometric_secondary_search;
  this->use_stand_still = use_stand_still;
  if(this->use_stand_still) {
    // All plans must be at least this long (required for avoiding obstacles moving towards a stationary UAV. Only works when stand-still-primitive exist)
    this->plan_duration_minimum = plan_duration_minimum;
  }
  else {
    this->plan_duration_minimum = 0.0;
  }

  this->closed_set_type = closed_set_type;
  exploredStates = 0;
  if(closed_set_type == "None") {
    exploredStates = new ClosedSetNone();
  }
  if(closed_set_type == "List") {
    exploredStates = new ClosedSetList();
  }
  if(closed_set_type == "Set") {
    exploredStates = new ClosedSetSet();
  }
  if(closed_set_type == "SetTime") {
    exploredStates = new ClosedSetSetTime();
  }
  if(closed_set_type == "SetWaitTime") {
    exploredStates = new ClosedSetSetWaitTime();
  }
  if(closed_set_type == "SetTimeTruncated") {
    exploredStates = new ClosedSetSetTimeTruncated();
  }
  if(closed_set_type == "SetTimeTruncatedHalf") {
    exploredStates = new ClosedSetSetTimeTruncated(2.0);
  }
  if(closed_set_type == "SetTimeTruncated10") {
    exploredStates = new ClosedSetSetTimeTruncated(10.0);
  }
  if(closed_set_type == "SetTimeRounded") {
    exploredStates = new ClosedSetSetTimeRounded();
  }
  if(exploredStates == 0) {
    exploredStates = new ClosedSetNone();
  }


  // Debug
  //this->print_obstacles();
  
  bool result = environment.load_primitives(path, primitive_amount_in_group, 
                                            use_stand_still, use_geometric_secondary_search, use_only_geometric_search);

  // Important, otherwise collision checking might not work
  environment.calculate_motion_primitive_hitboxes(uav_hitbox);
  
  bool abort_on_errors = false;
  abort_on_errors = true; // Debug
  result &= environment.debug_primitives(abort_on_errors);

  return result;

}

void LatticePlanner::debug_log(LP::TAG tag, std::string log) {
  if(write_debug_to_console) {
    if(tag == LP::ERROR) {
      ROS_ERROR_STREAM("Lattice Planner ERROR:\n" << log);
    }
    else if(tag == LP::WARN) {
      ROS_WARN_STREAM("Lattice Planner WARNING:\n" << log);
    }
    else
      std::cerr << log;
  }
}

bool LatticePlanner::is_root_state_in_collision() {
  bool start_in_allowed_volume = environment.in_allowed_volume(&this->rootState);
  bool start_in_collision = environment.in_collision(uav_hitbox, &this->rootState);
  return !start_in_allowed_volume || start_in_collision;
}


bool LatticePlanner::do_planning_cycle(SearchState from, 
                                       SearchState goal, 
                                       double planning_time, 
                                       vector<TrajectoryState> &planned_trajectory, 
                                       double start_time) {
  // Clear Memory
  profiler->reset();
  frontier.clear();
  frontier_lookup.clear();
  exploredStates->clear();
  solution_path.clear();
  memoryPool.reset();    
  environment.clear_predictions();
    
  profiler->mark("Memory cleanup");

  // Search Setup
  this->goalState = State(SearchState(goal.position.x(), goal.position.y(), goal.position.z(), 
                                      goal.velocity.x(), goal.velocity.y(), goal.velocity.z()),
                          -1);
  this->goalState.state.clean_up_numbers();
  this->rootState = State(SearchState(from.position.x(), from.position.y(), from.position.z(), 
                                      from.velocity.x(), from.velocity.y(), from.velocity.z()), 
                          start_time);
  this->rootState.state.clean_up_numbers();
  this->rootState.cost_g = 0;
  this->rootState.cost_f = this->rootState.cost_g + heuristic_fn(&this->rootState, &this->goalState);
  LP_DEBUG(LP::INFO, "Planning from (Memory adress) (" << &this->rootState << "): \n");
  LP_DEBUG(LP::INFO, "Planning from state= " << this->rootState.state << std::endl);
  LP_DEBUG(LP::INFO, "Planning to (Memory adress) (" << &this->goalState << "): \n");
  LP_DEBUG(LP::INFO, "Planning to state= " << this->goalState.state << std::endl);
  
  bool start_in_allowed_volume = environment.in_allowed_volume(&this->rootState);
  bool goal_in_allowed_volume = environment.in_allowed_volume(&this->goalState);
  bool start_in_collision = environment.in_collision(uav_hitbox, &this->rootState);
  bool goal_in_collision = environment.in_collision(uav_hitbox, &this->goalState, true); // Static obstacle collision only

  profiler->mark("Search setup");
  if(!start_in_allowed_volume || !goal_in_allowed_volume || start_in_collision || goal_in_collision) {
    
    LP_DEBUG(LP::INFO, "> No path was found to goal <\n");
    LP_DEBUG(LP::INFO, ">  Primitive classes: " << environment.get_primitives().size() << "\n");
    
    if(!start_in_allowed_volume) {
      LP_DEBUG(LP::INFO, "> ! Starting location is OUTSIDE of allowed area - no plan possible \n");
    } 
    else if(start_in_collision) {
      LP_DEBUG(LP::INFO, "> ! Starting location is in collision - no plan possible \n");
    }
    
    if(!goal_in_allowed_volume) {
      LP_DEBUG(LP::INFO, "> ! Goal location is OUTSIDE of allowed area - no plan possible \n");
    } 
    else if(goal_in_collision) {
      LP_DEBUG(LP::INFO, "> ! Goal location is in collision - no plan possible \n");
    }
    
    LP_DEBUG(LP::INFO, *profiler);
    return false;
  }

  double search_time_1 = planning_time;
  double search_time_2 = planning_time*2;
  bool search_success = false;
  
  int evaluated_nodes_search_1 = 0;
  int evaluated_nodes_search_2 = 0;

  LP_DEBUG(LP::INFO, "Expanding frontier from state - START\n");
  // Search 1 preparations
  // Push primitives reachable from the current state onto the frontier
  search_success = expand_frontier_from_state(&this->rootState,
                                              start_time,
                                              frontier,
                                              exploredStates);
  profiler->mark("Graph search 1 - preparations");
  if(search_success){
    LP_DEBUG(LP::INFO, "Solution found (single action) \n");
  }
  LP_DEBUG(LP::INFO, "Expanding frontier from state - STOP\n");

  // Search 1
  if(!search_success) {
    LP_DEBUG(LP::INFO, "Primary Search - START\n");
    search_success = this->primary_search(search_time_1, start_time, this->plan_duration_minimum,
                                          environment.get_primary_primitive_map(),
                                          frontier,
                                          exploredStates);
    LP_DEBUG(LP::INFO, "Primary Search - STOP\n");
    profiler->mark("Graph search 1");
    if(search_success) {
      LP_DEBUG(LP::INFO, "Solution found (search 1) \n");
    }
    else {
      LP_DEBUG(LP::INFO, "Solution not found (search 1) \n");
    }
  }
  evaluated_nodes_search_1 = exploredStates->size();

  if(!search_success && use_geometric_secondary_search) {
    // Search 2 preparations
    frontier_lookup.clear();
    secondary_frontier.clear();
    double max_time = -1;
    if(frontier.empty()) {
      for(State * state: exploredStates->get_states()) {
        max_time = std::max(state->time, max_time);
      }
      LP_DEBUG(LP::WARN, ">> primary frontier empty, picking a set of states with the maximum time = " << max_time << " (< min_time) \n");
      for(State * state: exploredStates->get_states()) {
          if(state->time == max_time) {
            if(state->actionID != -1) {
              add_to_frontier(secondary_frontier, state);
            }
          }
      }
    }
    else {
      for(FrontierContainer::iterator state = frontier.begin();
          state != frontier.end(); state++) {
          if((*state)->time >= start_time + this->plan_duration_minimum) {
            if((*state)->actionID != -1) {
              add_to_frontier(secondary_frontier, *state);
            }
          }
          else {
            max_time = std::max((*state)->time, max_time);
          }
      }
      if(secondary_frontier.empty()) {
        LP_DEBUG(LP::WARN, ">> secondary frontier empty, picking a set of states with the maximum time = " << max_time << " (< min_time) \n");
        for(FrontierContainer::iterator state = frontier.begin();
          state != frontier.end(); state++) {
          if((*state)->time == max_time) {
            if((*state)->actionID != -1) {
              add_to_frontier(secondary_frontier, *state);
            }
          } 
        }
      }
    }
    
    LP_DEBUG(LP::INFO, "Frontier size:        " << secondary_frontier.size() << "\n");
    LP_DEBUG(LP::INFO, "Frontier lookup size: " << frontier_lookup.size() << "\n");
    if(secondary_frontier.empty()) {
      LP_DEBUG(LP::ERROR, ">> secondary frontier is still empty! Something is very wrong...");
    }
    profiler->mark("Graph search 2 - preparations");

    // Search 2
    search_success = this->secondary_search(search_time_2, start_time,
                                            environment.get_secondary_primitive_map(),
                                            secondary_frontier, // To be changed to frontier2
                                            exploredStates);
    profiler->mark("Graph search 2");
    if(search_success) {
      LP_DEBUG(LP::INFO, "Solution found (search 2) \n");
    }
    else {
      LP_DEBUG(LP::INFO, "Solution not found (search 2) \n");
    }
  }
  evaluated_nodes_search_2 = exploredStates->size() - evaluated_nodes_search_1;

  if(search_success) {
    LP_DEBUG(LP::INFO, "> path to goal found <" << std::endl);
  }
  else {
    ROS_WARN("No plan to goal found. Picking the one which takes us closest to the goal.");
    ROS_FATAL("No plan to goal found. AND NOT IMPLEMENTED YET..");
    if(false) {
      State * best_destination;
      if(use_geometric_secondary_search) {
        best_destination = find_best_destination(secondary_frontier, start_time + this->plan_duration_minimum);
      }
      else {
        best_destination = find_best_destination(frontier, start_time + this->plan_duration_minimum);
      }

      if(best_destination == 0) {
        LP_DEBUG(LP::INFO, "> path to goal not found <" << std::endl); 
      }
      else {      
        LP_DEBUG(LP::INFO, ">> Picked state as goal with the maximum time = " << best_destination->time << " (< min_time) and which is closest to the goal \n");
        this->goalState = *best_destination;
        search_success = true;
      }
      profiler->mark("Search horizon");
    }
  }

  if(search_success) {
    generate_solution_path();
    generate_solution_trajectory(planned_trajectory);
    profiler->mark("Generate solution trajectory");
  }

  // Search Info
  LP_DEBUG(LP::INFO, ">  Primitive classes: " << environment.get_primitives().size() << "\n");
  LP_DEBUG(LP::INFO, ">  Evaluated nodes:              " << exploredStates->size() << "\n");
  LP_DEBUG(LP::INFO, ">  Evaluated (Primary) nodes:    " << evaluated_nodes_search_1 << "\n");
  LP_DEBUG(LP::INFO, ">  Frontier (Primary) size:      " << frontier.size() << "\n");
  LP_DEBUG(LP::INFO, ">  Evaluated (Secondary) nodes:  " << evaluated_nodes_search_2 << "\n");
  LP_DEBUG(LP::INFO, ">  Frontier (Secondary)  size:   " << secondary_frontier.size() << "\n");
  if(search_success) {
    LP_DEBUG(LP::INFO, "> Solution length:   " << solution_path.size() << "\n");
    LP_DEBUG(LP::INFO, "> Trajectory length: " << planned_trajectory.size() << "\n");
  }
  
  // Debug
  //print_explored_states();
  //LP_DEBUG(LP::INFO, "\n");
  //print_frontier();
  LP_DEBUG(LP::INFO, "\n");
  LP_DEBUG(LP::INFO, "Frontier (primary) size:        " << frontier.size() << "\n");
  LP_DEBUG(LP::INFO, "Frontier (secondary) size:      " << secondary_frontier.size() << "\n");
  LP_DEBUG(LP::INFO, "Frontier lookup size:           " << frontier_lookup.size() << "\n");

  LP_DEBUG(LP::INFO, *profiler);
  return search_success;
}

State * LatticePlanner::find_best_destination(FrontierContainer & frontier, double min_time) {
  if(frontier.empty())
    return 0;

  ClosestToGoalLessThan lessThan;
  State * best_destination;
  std::vector<State*> possible_destinations;
  double max_time = -1;
  for(FrontierContainer::iterator state = frontier.begin();
      state != frontier.end(); state++) {
      if((*state)->time >= min_time) {
        possible_destinations.push_back(*state);
      } 
      else {
        max_time = std::max((*state)->time, max_time);
      }
  }
  if(possible_destinations.empty()) {
    for(FrontierContainer::iterator state = frontier.begin();
      state != frontier.end(); state++) {
      if((*state)->time == max_time) {
        possible_destinations.push_back(*state);
      } 
    }
  }
  if(possible_destinations.empty()) {
    LP_DEBUG(LP::ERROR, "possible_destinations is empty, this should not be possible here!");
    ROS_FATAL("possible_destinations is empty, this should not be possible here!");
  }
  best_destination = possible_destinations.front();
  for(int n = 1; n < possible_destinations.size(); n++) {
    if(lessThan(possible_destinations[n], best_destination))
      best_destination = possible_destinations[n];
  }
  return best_destination;
}


bool LatticePlanner::primary_search(double planning_time, 
                                     double start_time,
                                     double plan_duration_minimum,
                                     std::vector<std::vector<int> > & neighbors,
                                     FrontierContainer & frontier,
                                     ClosedSet * closed_set) {
  
  double time = 0;
  double minimum_plan_time = plan_duration_minimum + start_time;
  bool is_goal_found = false;
  double goal_state_time = -1;
  ros::Time startTime = ros::Time::now();
  while (time < planning_time && !is_goal_found && !is_frontier_empty(frontier))
  {

    //ROS_INFO_STREAM_THROTTLE(rate, "   frontier.size(): " << frontier.size() );
    State * state = pop_from_fontier(frontier);
    //ROS_INFO_STREAM_THROTTLE(rate, "   state:  " << state );
    //ROS_INFO_STREAM_THROTTLE(rate, "   *state: " << *state );
    //LP_DEBUG(LP::ERROR, "Decreased frontier: " << frontier.size() << "\n");
    //LP_DEBUG(LP::ERROR, "neighbors[" << state->actionID << "].size(): " << neighbors[state->actionID].size() << "\n");
    closed_set->set_visited(state);
    
    if(is_goal(state) && state->time >= minimum_plan_time) {
      goalState = *state;
      goalState.cost_f = goalState.cost_g;
      closed_set->set_visited(&goalState);
      is_goal_found = true;
      break;
    }
        
    AssertMsg(state->actionID >= 0, "state->actionID: " << state->actionID);
    AssertMsg(neighbors.size() > state->actionID, "neighbors.size(): " << neighbors.size() << ", state->actionID: " << state->actionID);
    for(int n = 0; n < neighbors[state->actionID].size(); n++) {
      int ID = neighbors[state->actionID][n];
      State newState = State(*state);
      environment.get_primitive(ID).transform_state(newState.state);
      newState.actionID = ID;
      newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
      newState.cost_f = newState.cost_g + heuristic_fn(&newState, &this->goalState);
      newState.parent = state; 
      newState.time += environment.get_primitive(ID).duration;
      newState.wait_time += environment.get_primitive(ID).wait_time;
      //newState.wait_time = std::min(newState.wait_time+environment.get_primitive(ID).wait_time, 6.0f);
           
      if(closed_set->is_visited(&newState)) {
        continue;
      }
      double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState, start_time, false);
      if(isinf(proximity_cost)) {  // Hard constraint on collision
        continue;
      }
      // TODO: This needs to be tuned. 
      //LP_DEBUG(LP::INFO, "prox cost=" << proximity_cost << " previous g=" << newState.cost_g << " previous f=" << newState.cost_f << std::endl);
      //ROS_INFO_STREAM_COND(proximity_cost > 0.1, "prox cost=" << proximity_cost << " previous g=" << newState.cost_g << " previous f=" << newState.cost_f << std::endl);
      newState.cost_g += proximity_cost;
      newState.cost_f += proximity_cost;

      // Check if new state is a goal
      if(is_goal(state) && is_goal(&newState)) {
        if(state->time < minimum_plan_time) {
          newState.cost_g = state->cost_g; // The cost of standing still at the goal is 0
          newState.cost_f = newState.cost_g;
        }
      }
      
      if(make_unique_in_frontier(frontier, newState)) {
        State * newStatePtr = memoryPool.allocate();
        *newStatePtr = State(newState);
        newStatePtr->parent = state;
        add_to_frontier(frontier, newStatePtr);
        
        
        //LP_DEBUG(LP::ERROR, n << "] Frontier size:        " << frontier.size() << "\n");
        //LP_DEBUG(LP::ERROR, n << "] Frontier lookup size: " << frontier_lookup.size() << "\n");
      }
    }   
    
    if(write_debug_to_console) {
      ROS_INFO_STREAM_THROTTLE(0.1, 
           " LatticePlanner::search (Running time: " << time << " / " << planning_time << " seconds)\n"
        << "  Evaluated nodes:      " << closed_set->size() << "\n"
        << "  Frontier size:        " << frontier.size()  << "\n"
        << "  Frontier lookup size: " << frontier_lookup.size() );
    }
    time = (ros::Time::now() - startTime).toSec();
  }

  bool time_out = !(time < planning_time);
  bool empty_frontier = is_frontier_empty(frontier);
  bool goal_state_found = goal_state_time >= 0;
  if(time_out) {
    LP_DEBUG(LP::INFO,"  Primary Planning failed: Time-out");
  }
  if(empty_frontier) {
    LP_DEBUG(LP::WARN,"  Primary Planning failed: Empty frontier (unreasonable in large world)");
  }
  if(!is_goal_found && goal_state_found) {  // Observe that minimum_plan_time is this->plan_duration_minimum + start_time
    LP_DEBUG(LP::WARN,"  Primary Planning failed: Goal state reached but minimum plan time not fulfilled (" 
                     << goal_state_time << " of required " << minimum_plan_time << ").");
  }

  return is_goal_found;  
}

bool LatticePlanner::secondary_search(double planning_time, 
                                      double start_time,
                                      std::vector<std::vector<int> > & neighbors,
                                      FrontierContainer & frontier,
                                      ClosedSet * closed_set) {
  
  double time = 0;
  bool is_goal_found = false;
  ros::Time startTime = ros::Time::now();
  while (time < planning_time && !is_goal_found && !is_frontier_empty(frontier))
  {
    State * state = pop_from_fontier(frontier);
    //LP_DEBUG(LP::ERROR, "Decreased frontier: " << frontier.size() << "\n");
    //LP_DEBUG(LP::ERROR, "neighbors[" << state->actionID << "].size(): " << neighbors[state->actionID].size() << "\n");
    closed_set->set_visited(state);
    
    // Check if new state is a goal
    if(is_goal(state)) 
    {
      LP_DEBUG(LP::INFO, "Goal state has been found in secondary search!" << "\n");
      goalState = *state;
      goalState.cost_f = goalState.cost_g;
      is_goal_found = true;
      break;
    }
    
    AssertMsg(state->actionID < neighbors.size(), state->actionID << " < " << neighbors.size());  //Debug
    AssertMsg(state->actionID >= 0, "state->actionID: " << state->actionID << ", state: " << *state);  //Debug
    for(int n = 0; n < neighbors[state->actionID].size(); n++) {
      int ID = neighbors[state->actionID][n];

      //LP_DEBUG(LP::ERROR, ID << "\n");

      State newState = State(*state);
      //newstate.state.velocity.x() = newstate.state.velocity.y() = newstate.state.velocity.z() = 0.0;
      environment.get_primitive(ID).transform_state(newState.state);
      newState.actionID = ID;
      newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
      newState.cost_f = newState.cost_g + heuristic_fn2(&newState, &this->goalState);
      newState.parent = state; 
      newState.time += environment.get_primitive(ID).duration;
      
      if(ID >= environment.get_secondary_start_index()) {
        newState.wait_time = -10;
      }
      else {
        newState.wait_time = state->wait_time;  // Keep the wait time for transition primitives
      }
      
      if(closed_set->is_visited(&newState)) {
        continue;
      }
      double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState, 0.0, true);
      if(isinf(proximity_cost)) {  // Hard constraint on collision
        continue;
      }

      if(make_unique_in_frontier(frontier, newState)) {
        State * newStatePtr = memoryPool.allocate();
        *newStatePtr = State(newState);
        newStatePtr->parent = state;
        add_to_frontier(frontier, newStatePtr);
      }
    }

    if(write_debug_to_console) {
      ROS_INFO_STREAM_THROTTLE(0.1, 
           " LatticePlanner::search (Running time: " << time << " / " << planning_time << " seconds)\n"
        << "  Evaluated nodes:      " << closed_set->size() << "\n"
        << "  Frontier size:        " << frontier.size()  << "\n"
        << "  Frontier lookup size: " << frontier_lookup.size() );
    }
    time = (ros::Time::now() - startTime).toSec();
  }

  bool time_out = !(time < planning_time);
  bool empty_frontier = is_frontier_empty(frontier);
  if(time_out) {
    LP_DEBUG(LP::INFO,"  Secondary Planning failed: Time-out");
  }
  if(empty_frontier) {
    LP_DEBUG(LP::ERROR,"  Secondary Planning failed: Empty frontier (unreasonable in large world)");
  }

  return is_goal_found;  
}

bool LatticePlanner::expand_frontier_from_state(State * root,
                                                double start_time,
                                                FrontierContainer & frontier,
                                                ClosedSet * closed_set) {
  //double minimum_plan_time = this->plan_duration_minimum + start_time;
  bool is_goal_found = false;
  State * state = root;
  closed_set->set_visited(root);
  std::vector<State*> initial_states;
  
  for(int ID = 0; ID < environment.get_secondary_start_index(); ID++) {
    if(possible_to_expand(*state, ID)) {
      State newState = State(*state);
      environment.get_primitive(ID).transform_state(newState.state);
      newState.actionID = ID;
      newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
      newState.cost_f = newState.cost_g + heuristic_fn(&newState, &this->goalState);
      newState.parent = 0;
      newState.time += environment.get_primitive(ID).duration;  
      newState.wait_time += environment.get_primitive(ID).wait_time;
                
      if(closed_set->is_visited(&newState) ) {
        continue;
      }
      double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState);
      if(isinf(proximity_cost)) {  // Hard constraint on collision
        continue;
      }
      //LP_DEBUG(LP::INFO, "prox cost=" << proximity_cost << " previous g=" << newState.cost_g << " previous f=" << newState.cost_f << std::endl); 
      newState.cost_g += proximity_cost;
      newState.cost_f += proximity_cost;

      if(make_unique_in_frontier(frontier, newState)) {
        State * newStatePtr = memoryPool.allocate();
        *newStatePtr = State(newState);
        newStatePtr->parent = state;
        add_to_frontier(frontier, newStatePtr);
      }
      
    }
  }  
  
  bool empty_frontier = is_frontier_empty(frontier);
  if(empty_frontier) {
    LP_DEBUG(LP::ERROR,"  Planning preparation failed: Empty frontier (unreasonable unless stuck)");
  }
  return is_goal_found;
}

void LatticePlanner::get_longest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory) {
  plan.clear();
  trajectory.clear();

  if(longest_collision_free_goal == 0)
    return;

  State goal_state = this->goalState;
  this->goalState = *longest_collision_free_goal;
  this->generate_solution_path();
  plan = clone_solution_path();
  generate_solution_trajectory(trajectory);

  // Clean up
  this->solution_path.clear();
  this->goalState = goal_state;
}

void LatticePlanner::get_closest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory) {
  plan.clear();
  trajectory.clear();

  if(closest_collision_free_goal == 0)
    return;

  State goal_state = this->goalState;
  this->goalState = *closest_collision_free_goal;
  this->generate_solution_path();
  plan = clone_solution_path();
  generate_solution_trajectory(trajectory);

  // Clean up
  this->solution_path.clear();
  this->goalState = goal_state;
}

//TODO: Maybe bad implementation?
bool LatticePlanner::goal_found(){
  if(this->goalState.is_root())
    return false;
  else
    return true;
}

void LatticePlanner::print_frontier() {

  LP_DEBUG(LP::INFO, "Printing frontier states:\n");
  for(FrontierContainer::iterator front = this->frontier.begin(); front != this->frontier.end(); front++)
  {
    LP_DEBUG(LP::INFO, "(" << *front << "): " << *(*front) << std::endl); 
  }
}
  
void LatticePlanner::print_explored_states() {

  LP_DEBUG(LP::INFO, "Printing explored states:\n");
  LP_DEBUG(LP::INFO, exploredStates->to_string());
}


void LatticePlanner::print_trajectory(std::vector<TrajectoryState> & trajectory) {
  int n = 0;  
  for(std::vector<TrajectoryState>::iterator state = trajectory.begin(); state != trajectory.end(); state++) {
    cout << n << "] " << *state << "\n";
    n++;
  }
}

void LatticePlanner::generate_solution_trajectory(std::vector<TrajectoryState> & trajectory) {
  LP_DEBUG(LP::INFO, "generate_solution_trajectory\n");

  int path_length = 0;
  for(std::vector<State*>::iterator state = solution_path.begin()+1; state != solution_path.end(); state++) {
    path_length += environment.get_primitives()[(*state)->actionID].trajectory_length();
  }

  trajectory.reserve(path_length+1);
  trajectory.clear();

  std::vector<State*>::iterator previous_state = solution_path.begin();
  double time = 0;
  for(std::vector<State*>::iterator state = solution_path.begin()+1; state != solution_path.end(); state++, previous_state++) {
    time = (*previous_state)->time;
    MotionPrimitive & primitive = environment.get_primitive((*state)->actionID);
    std::vector<TrajectoryState> sub_trajectory = primitive.apply((*previous_state)->state, time);

    /*
    // Debug
    LP_DEBUG(LP::INFO, time << "] From (" << *previous_state << "): " << **previous_state << "\n");
    LP_DEBUG(LP::INFO, time << "] To   (" << *state << "): " << **state << "\n");
    LP_DEBUG(LP::INFO, time << "] Primitive:               " << primitive);
    LP_DEBUG(LP::INFO, time << "] sub_trajectory:          ");
    print_trajectory(sub_trajectory);
    LP_DEBUG(LP::INFO, "  -----------------\n"); */
  
    if(state+1 == solution_path.end() || sub_trajectory.size() == 1){
      trajectory.insert(trajectory.end(), sub_trajectory.begin(), sub_trajectory.end());
    }else{
      trajectory.insert(trajectory.end(), sub_trajectory.begin(), sub_trajectory.end()-1);
    }
  }  
}

void LatticePlanner::goal_reached() {
  return solution_path.clear(); // Memory leak!
}


bool LatticePlanner::has_plan() {
  return !solution_path.empty();
}


/* Graph Search methods ------------------------ */

State* LatticePlanner::pop_from_fontier() {
  FrontierContainer::iterator state = this->frontier.begin();
  State * bestExploreState = *state;
  this->frontier.erase(state);
  this->frontier_lookup.erase(*bestExploreState);
  return bestExploreState;
}

bool LatticePlanner::possible_to_expand(State & state, int id) {

  if(std::abs(state.state.velocity.x() - environment.get_primitive(id).from.velocity.x()) < 0.1 && 
     std::abs(state.state.velocity.y() - environment.get_primitive(id).from.velocity.y()) < 0.1 &&
     std::abs(state.state.velocity.z() - environment.get_primitive(id).from.velocity.z()) < 0.1){  
    return true;
  }
  else{
    return false;
  }
}

bool LatticePlanner::is_goal(State * state) {
  return state->state == goalState.state;
}

void LatticePlanner::add_to_frontier(State * state) {
  FrontierContainer::iterator it = frontier.insert(state);
  std::pair<State,FrontierContainer::iterator> frontier_it(*state,it);
  frontier_lookup.insert(frontier_it);
}
 
bool LatticePlanner::frontier_empty() {
  return this->frontier.empty();
}   

void LatticePlanner::generate_solution_path() {
  std::vector<State*> & path = this->solution_path;
  State * state = &this->goalState;

  LP_DEBUG(LP::INFO, "generate_solution_path(1):\n");
  //LP_DEBUG(LP::INFO, "state: " << state << " | state->parent: " << state->parent << "\n");

  while(state != 0) {
    if(state == state->parent) {
      ROS_FATAL_STREAM("State is its own parent (LatticePlanner::generate_solution_path())");
      break;
    }

    path.push_back(state);
    state = state->parent;
    if(state != 0) {
      //LP_DEBUG(LP::INFO, "state: " << state << " | state->parent: " << state->parent << "\n");
    }
  }
  std::reverse(path.begin(), path.end());
  //LP_DEBUG(LP::INFO, "generate_solution_path(done):\n");
}

std::vector<State> LatticePlanner::clone_solution_path() {
  std::vector<State> path_clone;
  if(!solution_path.empty()) {
    path_clone.reserve(solution_path.size());
    path_clone.push_back(State(*solution_path[0]));
    for(int n = 1; n < solution_path.size(); n++) {
      path_clone.push_back(State(*solution_path[n]));
      path_clone[n].parent = &path_clone[n-1];
    }
  }
  return path_clone;  
}

bool LatticePlanner::isGoalInCollision() {
  return environment.in_collision(uav_hitbox, &this->goalState);
}

bool LatticePlanner::isStartInCollision() {
  return environment.in_collision(uav_hitbox, &this->rootState);
}


bool LatticePlanner::is_frontier_empty(FrontierContainer & frontier) {
  return frontier.empty();
}

State * LatticePlanner::pop_from_fontier(FrontierContainer & frontier) {
  FrontierContainer::iterator state = frontier.begin();
  State * bestExploreState = *state;
  frontier.erase(state);
  frontier_lookup.erase(*bestExploreState);
  return bestExploreState;
}

void LatticePlanner::add_to_frontier(FrontierContainer & frontier, State * state) {
  FrontierContainer::iterator it = frontier.insert(state);
  std::pair<State,FrontierContainer::iterator> frontier_it(*state,it);
  frontier_lookup.insert(frontier_it);
}

bool LatticePlanner::make_unique_in_frontier(FrontierContainer & frontier, State & state) {
  std::map<State,FrontierContainer::iterator>::iterator it = frontier_lookup.find(state);
  if(it == frontier_lookup.end()) { // The search state does not already exist in the frontier
    return true;
  }
  FrontierContainer::iterator frontier_it = it->second;
  if(state.cost_g < (*frontier_it)->cost_g) { // The search state already exist in the frontier, but the new one path is cheaper
    frontier_lookup.erase(it);
    frontier.erase(frontier_it);
    return true;
  }
  return false; // The search state already exist in the frontier and is not cheaper
}


bool is_plan_leading_to_goal(std::vector<State> & plan, State & goal) {
  if(plan.empty())
    return false;
  return plan.back() == goal;
}

bool is_plan_in_the_past(std::vector<State> & plan) {
  if(plan.empty())
    return false;
  return plan.back().time < 0.0;  
}

void LatticePlanner::update_octomap(octomap::OcTree* octree){
  environment.update_octomap(octree);
}

State LatticePlanner::create_new_state(State previous_state, int primitiveID) {
  MotionPrimitive primitive = environment.get_primitive(primitiveID);
  State new_state;
  new_state.time  = previous_state.time + primitive.duration;
  new_state.cost_g = previous_state.cost_g + primitive.cost;
  new_state.actionID = primitiveID;
  new_state.state.velocity = primitive.to.velocity;
  new_state.state.position = previous_state.state.position + (primitive.to.position - primitive.from.position);
  return new_state;
}

