#include "LatticePlanner.h"

bool LatticePlanner::initialize(std::string path, 
                  std::vector<int> primitive_amount_in_group, 
                  double uav_safety_radius, 
                  Profiler * profiler,
                  ros::NodeHandle nh,
                  bool use_stand_still,
                  bool use_geometric_secondary_search,
                  bool use_only_geometric_search,
                  double plan_duration_minimum,
                  std::string closed_set_type,
                  bool best_effort,
                  bool write_debug_to_console,
                  bool visualize_frontiers,
                  bool emergency_trajectories,
                  bool survival_planning) {

  
  ros::Time before_alloc = ros::Time::now();
  memoryPool.init(50000000);
  double after_alloc = (ros::Time::now() - before_alloc).toSec();

  ROS_FATAL_STREAM("Allocating memory took: " << after_alloc << " seconds");

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
  this->visualize_frontiers = visualize_frontiers;
  this->emergency_trajectories_ = emergency_trajectories;
  this->survival_planning_ = survival_planning;
  
  if(this->use_stand_still) 
  {
    // All plans must be at least this long (required for avoiding obstacles moving towards a stationary UAV. Only works when stand-still-primitive exist)
    this->plan_duration_minimum = plan_duration_minimum;
  }
  else {
    this->plan_duration_minimum = 0.0;
  }

  this->closed_set_type = closed_set_type;
  exploredStates = 0;
  ROS_WARN_STREAM("ClosedSetType is: " << closed_set_type );
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
  
  bool result = environment.load_primitives(path, primitive_amount_in_group, 
                                            use_stand_still, use_geometric_secondary_search, use_only_geometric_search);
 

  // Important, otherwise collision checking might not work. Must come after load_primitives()
  environment.calculate_motion_primitive_hitboxes(uav_hitbox);
  bool abort_on_errors = false;
  abort_on_errors = true; // Debug
  result &= environment.debug_primitives(abort_on_errors);

  // Visualization
  expand_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("frontier_expand", 1);
  primary_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("frontier_primary", 1);
  secondary_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("frontier_secondary", 1);
  solution_path_viz_pub_ = nh.advertise<visualization_msgs::Marker>("solution_path", 1);

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
                                       double start_time,
                                       TrajectoryState collision_state,
                                       double time_until_next_collision,
                                       bool current_plan_valid) {
  
  
  double start_time1 = ros::Time::now().toSec();
  // ----------------------- Clear Memory -----------------------
  // ############################################################

  profiler->reset();
  frontier.clear();
  frontier_lookup.clear();
  exploredStates->clear();
  solution_path.clear();
  memoryPool.reset();    
  environment.clear_predictions();
  profiler->mark("Memory cleanup");

  // ----------------------- Search Setup -----------------------
  // ############################################################
  this->goalState = State(SearchState(goal.position.x(), goal.position.y(), goal.position.z(), 
                                      goal.velocity.x(), goal.velocity.y(), goal.velocity.z()),
                          -1);
  this->goalState.state.clean_up_numbers(); //Create true zeros if needed
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
  bool start_in_collision = environment.in_collision(uav_hitbox, &this->rootState, true); //Static obstacle check only
  bool goal_in_collision = environment.in_collision(uav_hitbox, &this->goalState, true); // Static obstacle check only

  profiler->mark("Search setup");

  bool search_success = false;
  bool primary_search_success = false;
  bool secondary_search_success = false;  
  int evaluated_nodes_search_1 = 0;
  int evaluated_nodes_search_2 = 0;
  State * backup_destination = 0;

  if(!start_in_allowed_volume || !goal_in_allowed_volume || start_in_collision || goal_in_collision) 
  {

    LP_DEBUG(LP::INFO, ">  Primitive classes: " << environment.get_primitives().size() << "\n");
    
    if(!start_in_allowed_volume) {
      LP_DEBUG(LP::INFO, "> ! Location of expansion is OUTSIDE of allowed area - no plan possible \n");
    } 
    else if(start_in_collision) {
      LP_DEBUG(LP::INFO, "> ! Location of expansion is in collision - no plan possible \n");
    }
    
    if(!goal_in_allowed_volume) {
      LP_DEBUG(LP::INFO, "> ! Goal location is OUTSIDE of allowed area - no plan possible \n");
    } 
    else if(goal_in_collision) {
      LP_DEBUG(LP::INFO, "> ! Goal location is in collision - no plan possible \n");
    }
    
    LP_DEBUG(LP::INFO, *profiler);
    return search_success;
  }
 
  // ----------------------- Search 1 Preperations  -----------------------
  // ######################################################################

  LP_DEBUG(LP::INFO, "Expanding frontier from root - START \n");
  
  search_success = expand_frontier_from_state(&this->rootState,
                                              start_time,
                                              frontier,
                                              exploredStates);
  profiler->mark("Graph search 1 - preparations");

  if(visualize_frontiers)
  {
    publish_visualization_frontier(frontier, LP::EXPAND);
  }
  
  if(!search_success)
  {
    LP_DEBUG(LP::INFO, "Frontier empty after expanding from root \n");
  }
  
  LP_DEBUG(LP::INFO, "Expanding frontier from state - STOP\n");

  LP_DEBUG(LP::INFO, "Primary Frontier size:        " << frontier.size() << "\n");
  LP_DEBUG(LP::INFO, "Primary Frontier lookup size: " << frontier_lookup.size() << "\n");

  double search_time_1{};
  double search_time_2{};
  double elapsed = ros::Time::now().toSec() - start_time1;
  
  if(use_geometric_secondary_search)
  {
    search_time_1 = (planning_time - elapsed) * (1.0/3.0);
    search_time_2 = (planning_time - elapsed) * (2.0/3.0);
  }
  else
  {
    search_time_1 = planning_time - elapsed;
    search_time_2 = 0;
  }

  // ----------------------- Search 1 -----------------------
  // ############################################################
  
  FrontierContainer frontier_backup;
  if(search_success) 
  {
    LP_DEBUG(LP::INFO, "Primary Search - START - Searching for " << search_time_1 << " seconds \n\n");
    
    // High resolution search using primary primitive map
    search_success = this->primary_search(search_time_1, start_time, this->plan_duration_minimum,
                                          environment.get_primary_primitive_map(),
                                          frontier,
                                          exploredStates);
    
    primary_search_success = search_success;
    frontier_backup = frontier;
    LP_DEBUG(LP::INFO, "Primary Search - STOP\n");
    profiler->mark("Graph search 1");
    
    if(search_success) 
    {
      LP_DEBUG(LP::INFO, "Solution found (search 1) \n");
      LP_DEBUG(LP::INFO, "SEARCH RESULT: FULL \n");
    }
    else 
    {
      LP_DEBUG(LP::INFO, "Solution not found (search 1) \n");
    }
  }

  evaluated_nodes_search_1 = exploredStates->size();


  if(visualize_frontiers)
  {
    publish_visualization_frontier(frontier, LP::PRIMARY);
  }

  FrontierContainer secondary_frontier_backup;
  double start_time2 = ros::Time::now().toSec();

  if(!search_success && use_geometric_secondary_search) 
  {
  
  // ----------------------- Search 2 Preperations  -----------------------
  // ######################################################################

    frontier_lookup.clear();
    secondary_frontier.clear();
    double max_time = -1;
    if(frontier.empty()) 
    {
      // In explored states, collect the maximum time
      for(State * state: exploredStates->get_states()) 
      {
        max_time = std::max(state->time, max_time);
      }

      LP_DEBUG(LP::WARN, ">> primary frontier empty, picking a set of states from ClosedSet with the maximum time = " << max_time << " (> min_time)" << std::endl);
      for(State * state: exploredStates->get_states()) 
      {
        if(state->time == max_time) 
        {
          if(state->actionID != -1) 
          {
            add_to_frontier(secondary_frontier, state);
          }
        }
      }
    }
    else 
    {
      for(FrontierContainer::iterator state = frontier.begin(); state != frontier.end(); state++) 
      {
          if((*state)->time >= start_time + this->plan_duration_minimum) 
          {
            if((*state)->actionID != -1) 
            {
              add_to_frontier(secondary_frontier, *state);
            }
          }
          else 
          {
            max_time = std::max((*state)->time, max_time);
          }
      }
      if(secondary_frontier.empty()) 
      {
        LP_DEBUG(LP::INFO, "SEARCH RESULT: EPHEMERAL \n");
        LP_DEBUG(LP::WARN, ">> secondary frontier empty, picking a set of states with the maximum time = " << max_time << " (> min_time) \n");
        for(FrontierContainer::iterator state = frontier.begin(); state != frontier.end(); state++)
        {
          if((*state)->time == max_time) 
          {
            if((*state)->actionID != -1) 
            {
              add_to_frontier(secondary_frontier, *state);
            }
          } 
        }
      }
      else
      {
        LP_DEBUG(LP::INFO, "SEARCH RESULT: REDUCED \n");
      }
    }
    
    LP_DEBUG(LP::INFO, "Secondary Frontier size:        " << secondary_frontier.size() << "\n");
    LP_DEBUG(LP::INFO, "Secondary Frontier lookup size: " << frontier_lookup.size() << "\n");

    if(secondary_frontier.empty()) 
    {
      LP_DEBUG(LP::ERROR, ">> secondary frontier is still empty! Only possible if cornered...");
      return false;
    }

    if(visualize_frontiers)
    {
      publish_visualization_frontier(secondary_frontier, LP::SECONDARY);
    }

    profiler->mark("Graph search 2 - preparations");

    double elapsed2 = ros::Time::now().toSec() - start_time2;
    search_time_2 = search_time_2 - elapsed2;

    // ----------------------- Search 2 -----------------------
    // ########################################################

    LP_DEBUG(LP::INFO, "Secondary Search - START - Searching for " << search_time_2 << " seconds \n");

    // Low resolution search using secondary primitive map
    search_success = this->secondary_search(search_time_2, start_time,
                                            environment.get_secondary_primitive_map(),
                                            secondary_frontier, 
                                            exploredStates);
    
    secondary_search_success = search_success;
    secondary_frontier_backup = secondary_frontier;
    LP_DEBUG(LP::INFO, "Secondary Search - STOP\n");

    profiler->mark("Graph search 2");

    if(search_success) {
      LP_DEBUG(LP::INFO, "Solution found (search 2) \n");
    }
    else 
    {
      LP_DEBUG(LP::INFO, "Solution not found (search 2) \n");
    }
  }

  evaluated_nodes_search_2 = exploredStates->size() - evaluated_nodes_search_1;


  // ----------------------- Survival Planning ---------------------------------
  // ##########################################################################
  // IMPROVEMENT 2: Survival Planning

  bool searchingForTrueGoal = true;
  if(!search_success && !current_plan_valid && this->survival_planning_)
  {

    /*
    Note that this part of the code does not do a search. It utilizes the result from 
    search1 and search2 to make the best possible action when no plan can be found.
    Thus it does not require time from the planning-budget.
    */
    
    ROS_ERROR("SURVIVAL No plan to goal found in primary or secondary search!");
    LP_DEBUG(LP::INFO, "Primary backup Frontier size:     " << frontier_backup.size() << "\n");
    LP_DEBUG(LP::INFO, "Secondary backup Frontier size:   " << secondary_frontier_backup.size() << "\n");
    State * best_destination;

   
    if(is_state_initalized(collision_state) && !search_success && (time_until_next_collision > get_plan_duration_minimum())) 
    {
      ROS_ERROR_STREAM("Trying to plan to previous collision state as a temporary goal... since " << time_until_next_collision << " seconds > " << get_plan_duration_minimum() << " seconds");
      
      // If we have found a plan earlier that has become infeasible, plan to the point that became infeasbile (in other words the point of collision)
      // This should hopefully move us in the right direction, instead of moving towards the real goal in a euclidian (straight) fashion which might not be optimal.
      // If no such plan can be found, we resort to the euclidian one with the longst collision-free time.
      
      SearchState collision_search_state {collision_state.position.x(), 
                                          collision_state.position.y(), 
                                          collision_state.position.z(), 
                                          collision_state.velocity.x(), 
                                          collision_state.velocity.y(), 
                                          collision_state.velocity.z()};
  
      best_destination = find_best_destination(frontier_backup, start_time + this->plan_duration_minimum, collision_search_state); 

      if(best_destination == 0) 
      {
        LP_DEBUG(LP::ERROR, "No plan found when planning to previous collision state..." "\n"); 
      }
      else 
      {
        
        if(best_destination->time > start_time + this->plan_duration_minimum)
        {
          LP_DEBUG(LP::INFO, "SEARCH RESULT: LOCAL \n");
          LP_DEBUG(LP::INFO, "Plan found to previous collision state! \n");
        
        }
        else
        {
          LP_DEBUG(LP::INFO, "Plan found to previous collision state! \n");
        }

        this->tempGoalState = *best_destination;
        search_success = true;
        searchingForTrueGoal = false;
      }
    }

    // If we didn't find a previous collision point, try to plan as close as possible to the goal!
    if(!search_success)
    {
      ROS_ERROR("Resorting to trying to find the longest (time-wise) collision-free trajectory as close to the goal as possible...!");

      if(!is_frontier_empty(frontier_backup))
      {
        LP_DEBUG(LP::INFO, "Searching in Primary backup Frontier frontier... (collision-checked)" << "\n");
        best_destination = find_best_destination(frontier_backup, start_time + this->plan_duration_minimum, goal);
      }
      else 
      {
        if(use_geometric_secondary_search)
        {
          LP_DEBUG(LP::INFO, "Searching in Secondary backup Frontier frontier... (Not collision-checked)" << "\n");
          best_destination = find_best_destination(secondary_frontier_backup, start_time + this->plan_duration_minimum, goal);
        }
      }

      if(best_destination == 0) 
      {
        LP_DEBUG(LP::ERROR, "Did not find longest (time-wise) collision-free trajectory as close to the goal as possible. Resorting to stop_command()... <" "\n");
        search_success = false;
      }
      else 
      {
        LP_DEBUG(LP::INFO, "SEARCH RESULT: LOCAL \n");
        LP_DEBUG(LP::INFO, "Picked state as goal with the maximum time = " << best_destination->time << " (> min_time = " << start_time + this->plan_duration_minimum << ") and which is closest to the goal \n");
        this->tempGoalState = *best_destination;
        search_success = true;
        searchingForTrueGoal = false;
      }
    }
  }

  profiler->mark("Search horizon");
  
  if(search_success) 
  {
    if(searchingForTrueGoal)
    {
      generate_solution_path(&this->goalState);
    }
    else
    {
      generate_solution_path(&this->tempGoalState);
    }

    if(this->emergency_trajectories_ && !this->solution_path.empty())
    {
      publish_visualization_solution_path(this->solution_path);
    }
    generate_solution_trajectory(planned_trajectory);
  }
  
  profiler->mark("Generate solution trajectory");
  profiler->stop();

  return search_success;
}

void LatticePlanner::publish_visualization_frontier(FrontierContainer & frontier, LP::SEARCH_TYPE type)
{

  visualization_msgs::MarkerArray states;
  long id{};
  for(FrontierContainer::iterator state = frontier.begin(); state != frontier.end(); state++) 
  {
    visualization_msgs::Marker state_marker;

    state_marker.header.frame_id = "world";
    state_marker.header.stamp = ros::Time();
    state_marker.ns = "";
    state_marker.id = id++;
    state_marker.type = visualization_msgs::Marker::SPHERE;
    state_marker.action = visualization_msgs::Marker::ADD;
    state_marker.lifetime = ros::Duration(1);
    state_marker.pose.position.x = (*state)->state.position.x();
    state_marker.pose.position.y = (*state)->state.position.y();
    state_marker.pose.position.z = (*state)->state.position.z();
    state_marker.pose.orientation.x = 0.0;
    state_marker.pose.orientation.y = 0.0;
    state_marker.pose.orientation.z = 0.0;
    state_marker.pose.orientation.w = 1.0;
    state_marker.scale.x = 0.1;
    state_marker.scale.y = 0.1;
    state_marker.scale.z = 0.1;
    state_marker.color.a = 1.0;

    if(type == LP::EXPAND)
    {
      // ORANGE
      state_marker.color.r = 1.0;
      state_marker.color.g = 0.698;
      state_marker.color.b = 0.0;
    }
    else if (type == LP::PRIMARY)
    {
      // BLUE
      state_marker.color.r = 0.0;
      state_marker.color.g = 0.0;
      state_marker.color.b = 1.0;
    }
    else if (type == LP::SECONDARY)
    {
      // GREEN
      state_marker.color.r = 0.0;
      state_marker.color.g = 1.0;
      state_marker.color.b = 0.0;
    }
    else
    {
      LP_DEBUG(LP::WARN, "-- Trying to visualize unknown SEARCH_TYPE --");
    }
    
    
    
    states.markers.push_back(state_marker);
  }

  if(type == LP::EXPAND)
  {
    expand_viz_pub_.publish(states);
  }
  else if (type == LP::PRIMARY)
  {
    primary_viz_pub_.publish(states);
  }
  else if (type == LP::SECONDARY)
  {
    secondary_viz_pub_.publish(states);
  }
  else
  {
    LP_DEBUG(LP::WARN, "-- Trying to visualize unknown SEARCH_TYPE --");
  }
}


/*
Publish the states in the solution path.
*/
void LatticePlanner::publish_visualization_solution_path(std::vector<State*> solution_path)
{
  // Setup for marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "solution_path";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.pose.orientation.w = 1.0;

  for (int n = 0; n < solution_path.size(); n++)
  {
    State * state;
    state = solution_path[n];

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = (*state).state.position.x();
    point.y = (*state).state.position.y();
    point.z = (*state).state.position.z();
    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;

    marker.points.push_back(point);
    marker.colors.push_back(color);
  }

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  solution_path_viz_pub_.publish(marker);

}

/*
Returns a destination that where we are alive longer than min_time, or the longest one possible and that takes us
as close to the goal as possible. It searches for this point in the given frontier so be mindfull if you send in a 
collision-checked frontier or not. It will make sure to find a plan that is at least min_time long.
*/
State * LatticePlanner::find_best_destination(FrontierContainer & frontier, double min_time, SearchState goal) 
{

  if(frontier.empty())
  {
    LP_DEBUG(LP::INFO, "SEARCH RESULT: UNREACHABLE \n");
    LP_DEBUG(LP::WARN, "find_best_destination: Frontier is empty!");
    return 0;
  }

  ClosestToGoalLessThan lessThan(goal, this->use_stand_still);
  State * best_destination;
  std::vector<State*> possible_destinations;
  double max_time = -1;

  // Collect states where we are alive longer than min_time
  // otherwise, collect the longest one
  for(FrontierContainer::iterator state = frontier.begin(); state != frontier.end(); state++) 
  {

    if((*state)->time >= min_time) 
      {
        possible_destinations.push_back(*state);
      } 
      else 
      {
        max_time = std::max((*state)->time, max_time);
      }
  }

  // If no possible destinations exists where time >= min_time
  // Extract the longest plan we have
  if(possible_destinations.empty()) 
  {
    for(FrontierContainer::iterator state = frontier.begin(); state != frontier.end(); state++)
    {
      if((*state)->time == max_time) 
      {
        possible_destinations.push_back(*state);
      } 
    }
  }

  // Only possible if frontier was empty from the beginning
  if(possible_destinations.empty()) 
  {
    LP_DEBUG(LP::ERROR, "find_best_destination: Possible_destinations is empty, this should not be possible!");
    return 0;
  }

  // From the possible destinations, return the one
  // that takes us the closest to the goal
  best_destination = possible_destinations.front();  
  for(int n = 1; n < possible_destinations.size(); n++) 
  {
    if(lessThan(possible_destinations[n], best_destination))
    {
      best_destination = possible_destinations[n];
    }
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

    State * state = pop_from_frontier(frontier);
    closed_set->set_visited(state);
    
    if(is_goal(state) && state->time >= minimum_plan_time) 
    {
      goalState = *state;
      goalState.cost_f = goalState.cost_g;
      closed_set->set_visited(&goalState);
      is_goal_found = true;
      goal_state_time = goalState.time;
      break;
    }
        
    // Action-ID should always be larger than 0
    // Neighbors size to a MP should always larger than the action-ID
    AssertMsg(state->actionID >= 0, "state->actionID: " << state->actionID);
    AssertMsg(neighbors.size() > state->actionID, "neighbors.size(): " << neighbors.size() << ", state->actionID: " << state->actionID);

    // For each neighbour (possible next motion primitive) to the current motion primitive
    for(int n = 0; n < neighbors[state->actionID].size(); n++) 
    {
      int ID = neighbors[state->actionID][n];
      State newState = State(*state);
      environment.get_primitive(ID).transform_state(newState.state);
      newState.actionID = ID;
      newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
      newState.cost_f = newState.cost_g + heuristic_fn(&newState, &this->goalState);
      newState.parent = state; 
      newState.time += environment.get_primitive(ID).duration; //newState.time == Time-of-Arrival at newState
      newState.wait_time += environment.get_primitive(ID).wait_time;
           
      if(closed_set->is_visited(&newState)) 
      {
        continue;
      }

      //Collision check static and dynamic obstacles
      double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState, state->time, false, true);
      if(isinf(proximity_cost))
      {  
        // Collision will happen, reject newState and move forward in loop
        continue;
      }

      newState.cost_g += proximity_cost;
      newState.cost_f += proximity_cost;

      // Check if new state is a goal and accomodates time requirements
      // This is needed since we have wait-time states, note that both state and newState will
      // be goal-states then since we wait in state and end up in newState!
      if(is_goal(state) && is_goal(&newState))
      {
        if(state->time < minimum_plan_time) 
        {
          newState.cost_g = state->cost_g;
          newState.cost_f = newState.cost_g;
        }
      }
      
      if(make_unique_in_frontier(frontier, newState)) 
      {
        State * newStatePtr = memoryPool.allocate();
        *newStatePtr = State(newState);
        newStatePtr->parent = state;
        add_to_frontier(frontier, newStatePtr);
      }  
    }   
    
    /*
    if(write_debug_to_console) 
    {
      ROS_INFO_STREAM_THROTTLE(0.1, 
        "\n"
        << " Primary Search (Running time: " << time << " / " << planning_time << " seconds)\n"
        << " Evaluated nodes (primary):    " << closed_set->size() << "\n"
        << " Primary Frontier size:        " << frontier.size()  << "\n"
        << " Primary Frontier lookup size: " << frontier_lookup.size() );
    }
    */
    time = (ros::Time::now() - startTime).toSec();
  }

  bool time_out = !(time < planning_time);
  bool empty_frontier = is_frontier_empty(frontier);
  bool goal_duration = goal_state_time >= minimum_plan_time;

  
  if(time_out) {
    LP_DEBUG(LP::WARN,"  Primary Planning failed: Time-out");
  }
  if(empty_frontier) {
    LP_DEBUG(LP::WARN,"  Primary Planning failed: Empty frontier");
  }
  if(is_goal_found && !goal_duration) {
    LP_DEBUG(LP::WARN,"  Primary Planning failed: Goal state reached but minimum plan time not fulfilled (" << goal_state_time << " of required " << minimum_plan_time << ").");
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

    State * state = pop_from_frontier(frontier);
    closed_set->set_visited(state);
    
    if(is_goal(state)) 
    {
      goalState = *state;
      goalState.cost_f = goalState.cost_g;
      is_goal_found = true;
      break;
    }
    
    AssertMsg(state->actionID < neighbors.size(), state->actionID << " < " << neighbors.size());
    AssertMsg(state->actionID >= 0, "state->actionID: " << state->actionID << ", state: " << *state);

    
    for(int n = 0; n < neighbors[state->actionID].size(); n++) 
    {
      int ID = neighbors[state->actionID][n];
      State newState = State(*state);
      environment.get_primitive(ID).transform_state(newState.state);
      newState.actionID = ID;
      newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
      newState.cost_f = newState.cost_g + heuristic_fn2(&newState, &this->goalState);
      newState.parent = state; 
      newState.time += environment.get_primitive(ID).duration;
      
      // Keep the wait time for transition primitives
      if(ID >= environment.get_secondary_start_index()) 
      {
        newState.wait_time = -10;
      }
      else 
      {
        newState.wait_time = state->wait_time;  
      }
      
      if(closed_set->is_visited(&newState)) 
      {
        continue;
      }

      // Only collision checking the static environment!
      double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState, state->time, true, false);
      if(isinf(proximity_cost)) 
      {  
        // If collision will happen, reject the state and continue the loop.
        continue;
      }

      if(make_unique_in_frontier(frontier, newState)) 
      {
        State * newStatePtr = memoryPool.allocate();
        *newStatePtr = State(newState);
        newStatePtr->parent = state;
        add_to_frontier(frontier, newStatePtr);
      }
    }

    /*
    if(write_debug_to_console) 
    {
      ROS_INFO_STREAM_THROTTLE(0.1, 
        "\n"
        << " Secondary Search (Running time: " << time << " / " << planning_time << " seconds)\n"
        << " Evaluated nodes (primary):      " << closed_set->size() << "\n"
        << " Primary Frontier size:          " << frontier.size()  << "\n"
        << " Primary Frontier lookup size:   " << frontier_lookup.size() );
    }
    */

    time = (ros::Time::now() - startTime).toSec();
  }

  bool time_out = !(time < planning_time);
  bool empty_frontier = is_frontier_empty(frontier);
  
  if(time_out) 
  {
    LP_DEBUG(LP::WARN,"  Secondary Planning failed: Time-out \n");
  }
  if(empty_frontier) 
  {
    LP_DEBUG(LP::WARN,"  Secondary Planning failed: Empty frontier \n");
  }

  return is_goal_found;  
}


bool LatticePlanner::expand_emergency_from_state(SearchState from, 
                                                  SearchState goal, 
                                                  double planning_time, 
                                                  vector<TrajectoryState> &planned_trajectory, 
                                                  double start_time,
                                                  TrajectoryState collision_state,
                                                  double time_until_next_collision,
                                                  bool current_plan_valid)
{

  double start_time1 = ros::Time::now().toSec();
  // ----------------------- Clear Memory -----------------------
  // ############################################################
  //profiler->reset();
  frontier.clear();
  frontier_lookup.clear();
  exploredStates->clear();
  solution_path.clear();
  memoryPool.reset();    
  environment.clear_predictions();
  //profiler->mark("Memory cleanup");

  // ----------------------- Search Setup -----------------------
  // ############################################################
  this->rootState = State(SearchState(from.position.x(), from.position.y(), from.position.z(), 
                                      from.velocity.x(), from.velocity.y(), from.velocity.z()), 
                          start_time);
  
  this->goalState = State(SearchState(goal.position.x(), goal.position.y(), goal.position.z(), 
                                      goal.velocity.x(), goal.velocity.y(), goal.velocity.z()),
                          -1);
  this->goalState.state.clean_up_numbers(); //Create true zeros if needed
  
  this->rootState.state.clean_up_numbers();
  this->rootState.cost_g = 0;
  this->rootState.cost_f = this->rootState.cost_g + heuristic_fn(&this->rootState, &this->goalState);
  
  LP_DEBUG(LP::INFO, "Planning from (Memory adress) (" << &this->rootState << "): \n");
  LP_DEBUG(LP::INFO, "Planning from state= " << this->rootState.state << std::endl);
  LP_DEBUG(LP::INFO, "Planning to (Memory adress) (" << &this->goalState << "): \n");
  LP_DEBUG(LP::INFO, "Planning to state= " << this->goalState.state << std::endl);
  
  bool start_in_allowed_volume = environment.in_allowed_volume(&this->rootState, 0.01);
  bool goal_in_allowed_volume = environment.in_allowed_volume(&this->goalState);
  bool start_in_collision = environment.in_collision(uav_hitbox, &this->rootState, true); //Static obstacle check only
  bool goal_in_collision = environment.in_collision(uav_hitbox, &this->goalState, true); // Static obstacle check only

  //profiler->mark("Search setup");

  if(!start_in_allowed_volume || !goal_in_allowed_volume || start_in_collision || goal_in_collision) 
  {
    
    LP_DEBUG(LP::INFO, "> No path was found to goal <\n");
    LP_DEBUG(LP::INFO, ">  Primitive classes: " << environment.get_primitives().size() << "\n");
    
    if(!start_in_allowed_volume) {
      LP_DEBUG(LP::INFO, "> ! Start location of expansion is OUTSIDE of allowed area - no plan possible \n");
      LP_DEBUG(LP::INFO, "> Start location: " << this->rootState.state << "\n");
      LP_DEBUG(LP::INFO, "> Allowed volume: " << environment.get_allowed_volume() << "\n");
    } 
    else if(start_in_collision) {
      LP_DEBUG(LP::INFO, "> ! Start location of expansion is in collision - no plan possible \n");
    }
    
    if(!goal_in_allowed_volume) {
      LP_DEBUG(LP::INFO, "> ! Goal location is OUTSIDE of allowed area - no plan possible \n");
    } 
    else if(goal_in_collision) {
      LP_DEBUG(LP::INFO, "> ! Goal location is in collision - no plan possible \n");
    }
    
    //LP_DEBUG(LP::INFO, *profiler);
    return false;
  }

  bool search_success = false; 
  int evaluated_nodes_search_1 = 0;
  int evaluated_nodes_search_2 = 0;

  // ----------------------- Search 1 Preperations  -----------------------
  // ######################################################################

  LP_DEBUG(LP::INFO, "EMERGENCY Expanding frontier from root - START \n");
  
  search_success = expand_frontier_from_state(&this->rootState,
                                              start_time,
                                              frontier,
                                              exploredStates);
  
  //profiler->mark("Graph search 1 - preparations");
 
  if(!search_success)
  {
    LP_DEBUG(LP::INFO, "EMERGENCY Frontier empty after expanding from root \n");
  }
  
  LP_DEBUG(LP::INFO, "EMERGENCY Expanding frontier from state - STOP\n");

  LP_DEBUG(LP::INFO, "EMERGENCY Primary Frontier size:        " << frontier.size() << "\n");
  LP_DEBUG(LP::INFO, "EMERGENCY Primary Frontier lookup size: " << frontier_lookup.size() << "\n");

  double search_time_1{};
  double search_time_2{};
  double elapsed = ros::Time::now().toSec() - start_time1;

  if(search_success)
  {
    ROS_INFO_STREAM("Successfully expanded from root in " << elapsed << " seconds");
  }
  else
  {
    ROS_INFO_STREAM("Unsuccessfully expanded from root in " << elapsed << " seconds");
  }

  // Pick on of the goals in the frontier
  if(search_success)
  {
    State * state_with_longest_time; //= pop_from_frontier(frontier);
    double longest_time = -1.0;

    for(FrontierContainer::iterator state = frontier.begin(); state != frontier.end(); state++) 
    {
      if((*state)->time >= longest_time)
      {
        longest_time = (*state)->time;
        state_with_longest_time = *state; //Deref iterator object to get underlying pointer.
      }
    }
    
    exploredStates->set_visited(state_with_longest_time);
    goalState = *state_with_longest_time;
    goalState.cost_f = goalState.cost_g;
  
    generate_solution_path(&this->goalState);
    generate_solution_trajectory(planned_trajectory);
  }

  return search_success;
}


/*
This functions expands the frontier, meaning that it collects all the motion primitives (271)
that goes from the root state to another newState. It will only collect those it can

 * possible expand to (given velocity contstraints)
 * Have not visited that state before
 * Does not have a proximity cost of infinity (meaning we have collision in that state)
 * Have not added that motion primitive to the frontier earlier
 
 It will modifier frontier in-place and fill it with all feasible frontiers.

*/
bool LatticePlanner::expand_frontier_from_state(State * root_state,
                                               double start_time,
                                               FrontierContainer & frontier,
                                               ClosedSet * closed_set) {
  
  State * state = root_state;
  closed_set->set_visited(root_state);

  //For all primitives
  for(int ID = 0; ID < environment.get_secondary_start_index(); ID++) 
  {

    // If we can follow primitive ID from root to newState
    if(possible_to_expand(*state, ID)) 
    {

      State newState = State(*state);
      environment.get_primitive(ID).transform_state(newState.state);
      newState.actionID = ID;
      newState.cost_g = state->cost_g + environment.get_primitive(ID).cost;
      newState.cost_f = newState.cost_g + heuristic_fn(&newState, &this->goalState);
      newState.parent = 0;
      newState.time += environment.get_primitive(ID).duration;  
      newState.wait_time += environment.get_primitive(ID).wait_time;
                
      if(closed_set->is_visited(&newState)) 
      {
        //If we have already visited newState, don't add it to the frontier
        continue;
      }

      // Collision check static and dynamic obtacles
      double proximity_cost = environment.evaluate_proximity_cost(uav_hitbox, &newState, start_time, false, true);
      if(isinf(proximity_cost)) 
      {  
        // Hard constraint on collision
        continue;
      }
      newState.cost_g += proximity_cost;
      newState.cost_f += proximity_cost;

      if(make_unique_in_frontier(frontier, newState)) 
      {
        State * newStatePtr = memoryPool.allocate();
        *newStatePtr = State(newState);
        newStatePtr->parent = state;
        add_to_frontier(frontier, newStatePtr);
      }
    }
  }
  
  
  bool empty_frontier = is_frontier_empty(frontier);

  if(empty_frontier) 
  {
    LP_DEBUG(LP::ERROR,"  Planning preparation failed: Empty frontier (unreasonable unless stuck or cornered)");
  }

  // If frontier is empty, return false meaning unsuccesful expansion
  // If frontier is not empty, return true, meaning we have a sucessful expansion.
  return !empty_frontier;
}


void LatticePlanner::get_longest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory) {
  plan.clear();
  trajectory.clear();

  if(longest_collision_free_goal == 0)
    return;

  this->generate_solution_path(longest_collision_free_goal);
  plan = clone_solution_path();
  generate_solution_trajectory(trajectory);

  // Clean up
  this->solution_path.clear();
}

void LatticePlanner::get_closest_collision_free_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory) {
  plan.clear();
  trajectory.clear();

  if(closest_collision_free_goal == 0)
    return;

  this->generate_solution_path(closest_collision_free_goal);
  plan = clone_solution_path();
  generate_solution_trajectory(trajectory);

  // Clean up
  this->solution_path.clear();
}

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

void LatticePlanner::generate_solution_trajectory(std::vector<TrajectoryState> & trajectory) 
{
  LP_DEBUG(LP::INFO, "Generating solution_trajectory \n");

  
  // Compute path length
  int path_length = 0;
  for(std::vector<State*>::iterator state = solution_path.begin()+1; state != solution_path.end(); state++) 
  {
    path_length += environment.get_primitives()[(*state)->actionID].trajectory_length();
  }

  trajectory.reserve(path_length+1);
  trajectory.clear();

  std::vector<State*>::iterator previous_state = solution_path.begin();
  double time = 0;
  for(std::vector<State*>::iterator state = solution_path.begin()+1; state != solution_path.end(); state++, previous_state++) 
  {
    time = (*previous_state)->time;
    MotionPrimitive & primitive = environment.get_primitive((*state)->actionID);
    
    // Adjust the primitive, both spatial and temporal
    std::vector<TrajectoryState> sub_trajectory = primitive.apply((*previous_state)->state, time);
  
    if(state + 1 == solution_path.end() || sub_trajectory.size() == 1)
    {
      trajectory.insert(trajectory.end(), sub_trajectory.begin(), sub_trajectory.end());
    }
    else
    {
      trajectory.insert(trajectory.end(), sub_trajectory.begin(), sub_trajectory.end()-1);
    }
  }  
}


bool LatticePlanner::has_plan() {
  return !solution_path.empty();
}


/* Graph Search methods ------------------------ */

State* LatticePlanner::pop_from_frontier() {
  FrontierContainer::iterator state = this->frontier.begin();
  State * bestExploreState = *state;
  this->frontier.erase(state);
  this->frontier_lookup.erase(*bestExploreState);
  return bestExploreState;
}

bool LatticePlanner::possible_to_expand(State & state, int id) 
{
  if(std::abs(state.state.velocity.x() - environment.get_primitive(id).from.velocity.x()) < 0.1 && 
     std::abs(state.state.velocity.y() - environment.get_primitive(id).from.velocity.y()) < 0.1 &&
     std::abs(state.state.velocity.z() - environment.get_primitive(id).from.velocity.z()) < 0.1)
  {  
    return true;
  }
  else
  {
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


/*
This function generates a solution path from a starting state (from where the DJI100 is currently)
to a specified ending state, given as input paramter. Note that the function modifies the internal solution_path variable 
and does not return a new path.
*/
void LatticePlanner::generate_solution_path(State * state) 
{
  std::vector<State*> & path = this->solution_path;
  LP_DEBUG(LP::INFO, "Generating_solution_path\n");

  while(state != 0) 
  {
    if(state == state->parent) 
    {
      ROS_FATAL_STREAM("State is its own parent (LatticePlanner::generate_solution_path())");
      break;
    }

    path.push_back(state);
    state = state->parent;
  }
  std::reverse(path.begin(), path.end());
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

State * LatticePlanner::pop_from_frontier(FrontierContainer & frontier) {
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

