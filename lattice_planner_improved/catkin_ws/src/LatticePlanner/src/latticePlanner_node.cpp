#include "latticePlanner_node.h"


LatticePlannerNode *lattice_planner_node; // Unfortunate global variable. Change
                                          // node to nodelet!



// ---------------------------------- MAIN LOOP -------------------------------------
/*
The main loop initializes the node, reads the parameters, and then goes into a loop where it waits for a new goal from the user or simulation.
It collects the data from the planning process and logs it into .csv files.
*/

int main(int argc, char **argv) 
{

  /*----------------- INIT NODE -------------------*/

  ros::init(argc, argv, "lattice_planner_node", ros::init_options::NoSigintHandler);
  
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  if (!std::numeric_limits<double>::is_iec559) 
  {
    ROS_ERROR("Inf usage requires IEEE 754 floats!");
    ros::shutdown();
  }

  
  // Define node
  ros::NodeHandle nh, private_nh("~");

  lattice_planner_node = new LatticePlannerNode();
  
  bool success = lattice_planner_node->initialize(nh, private_nh);

  if (!success) {
    ROS_ERROR("Initialization of lattice planner node failed");
    return 0;
  }

    /*----------------- INIT PARAMS -------------------*/

  double planning_time = 3.0;
  double replanning_time = 1.0;

  if (!private_nh.getParam("planning_time", planning_time))
  {
    ROS_WARN_STREAM("Using default planning time: " << planning_time);
  }
  
  if (!private_nh.getParam("replanning_time", replanning_time)) 
  {
    ROS_WARN_STREAM("Using default replanning time: " << replanning_time);
  }

  lattice_planner_node->logger.log("planning_time", 0.0, planning_time);
  lattice_planner_node->logger.log("replanning_time", 0.0, replanning_time);

  // Development tests
  std::string scenario_str;
  if (!private_nh.getParam("scenario", scenario_str)) 
  {
    ROS_ERROR("scenario not set (rosparam)");
    abort();
  }
  
  double scenario_parameter = 5;
  if (!private_nh.getParam("scenario_parameter", scenario_parameter)) 
  {
    ROS_ERROR("scenario_parameter not set (rosparam)");
  }
  
  lattice_planner_node->logger.log("scenario_parameter", 0.0, scenario_parameter);

  int seed = 0;
  if (!private_nh.getParam("seed", seed)) 
  {
    ROS_ERROR("seed not set (rosparam)");
  }
  lattice_planner_node->seed = seed;

  
  double location_tolerance = lattice_planner_node->location_tolerance_;
  if (!private_nh.getParam("location_tolerance", location_tolerance)) 
  {
    ROS_ERROR_STREAM("Using default location_tolerance: " << location_tolerance);
    abort();
  }
  lattice_planner_node->location_tolerance_ = location_tolerance;

  std::string test;
  if (!private_nh.getParam("test", test)) 
  {
    ROS_ERROR("test not set (rosparam)");
    abort();
  }

  std::string log_name;
  if (!private_nh.getParam("log_name", log_name)) 
  {
    ROS_WARN_STREAM("Not logging performance to file.");
  }

  std::string log_path;
  if (!private_nh.getParam("log_path", log_path)) {
    ROS_WARN_STREAM("Empty log path!");
  }

  if (log_name != "") 
  {
    std::string closed_set_type = "SetWaitTime";
    
    if (!private_nh.getParam("closed_set_type", closed_set_type)) 
    {
      ROS_WARN("closed_set_type not set (rosparam)");
    }

    std::stringstream ss;
    ss << log_name << "_" << scenario_str << "_" << closed_set_type << "_"
       << test << "_" << std::time(0); // no ".m"
    lattice_planner_node->logger.path = log_path;
    lattice_planner_node->logger.name = ss.str();
    ROS_INFO_STREAM("Logging to file: " << ss.str());
  }

  /*-------------------WRITE DATA TO FILE -------------------*/

  // Open file streams to write simulation data
  std::ofstream logfile, summaryfile;
  std::string homePath = std::getenv("HOME");
  logfile.open(homePath + "/data/logfile.csv");
  summaryfile.open(homePath + "/data/summary.csv");

  // Insert headers
  logfile << "Iteration, "
          << "Simulation Time [s], " 
          << "Planning Time Budget [s], " 
          << "Replanning Time Budget [s], "
          << "Search1 Time [s], "   
          << "Search2 Time [s], "
          << "Total Search Time [s], "
          << "Planning Cycle Time [s], "
          << "Heuristic Estimate to Goal [m], "
          << "Cost Start to Goal [m], "
          << "OpenSet [#State], "
          << "ClosedSet [#State], "
          << "Plan Size [#State], "
          << "Trajectory Plan Size [#TrajectoryState], "
          << "Time to reach Goal [s], "
          << "Cost for next primitive [m], "
          << "Time to execute next primitive [m], "
          << "Dynamic Obstacle Collisions [#], "
          << "Found plan? [bool], "
          << "Same as previous plan? [bool], "
          << "Primitive name [string], "
          << "Primitives Used [#Primitives], " << std::endl;

  summaryfile << "Total Simulation Time [s], "
              << "Planning Time Budget [s], " 
              << "Replanning Time Budget [s], "
              << "Accumulated Search1 Time [s], "   
              << "Accumulated Search2 Time [s], "
              << "Accumulated Total Search Time [s], "
              << "Accumulated Planning Cycle Time [s], "
              << "Accumulated Heuristic Estimate to Goal [m], "
              << "Accumulated Cost Start to Goal [m], "
              << "Accumulated OpenSet [#State], "
              << "Accumulated ClosedSet [#State], "
              << "Accumulated Plan Size [#State], "
              << "Accumulated Trajectory Plan Size [#TrajectoryState], " 
              << "Accumulated Travelled Distance [m], "
              << "Accumulated Primitive Time [s], " 
              << "Accumulated Dynamic Obstacle Collisions [#], " << std::endl;

  // Init logfile
  logfile << 0 << ", "
          << 0 << ", " 
          << 0 << ", " 
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << std::endl;


  // ----------------------- GOING INTO LATTICE PLANNER ----------------------------

  ROS_INFO_STREAM("WAITING FOR NAV_GOAL FROM USER OR SIMULATION...");
  ros::Rate loop_rate(10);
  lattice_planner_node->logger.log("loop_rate", 0.0, loop_rate.expectedCycleTime().toSec());

  double time_start = ros::Time::now().toSec();
  double new_goal_time = time_start;
  std::default_random_engine goal_rng{std::random_device{}()};
  goal_rng.seed(seed);

  // OctoMap
  int CountOpendMap = 0;
  bool enable_visualization_close_proximity{};
  bool enable_visualization_grid{};
  bool visualize_closed_set{};
  bool visualize_obstacles{};

  if (!private_nh.getParam("enable_visualization_close_proximity", enable_visualization_close_proximity)) {
    ROS_WARN("enable_visualization_close_proximity not set (rosparam)");
  }
  if (!private_nh.getParam("enable_visualization_grid", enable_visualization_grid)) {
    ROS_WARN("enable_visualization_grid not set (rosparam)");
  }

  if (!private_nh.getParam("visualize_closed_set", visualize_closed_set)) {
    ROS_WARN("visualize_closed_set not set (rosparam)");
  }
  
   if (!private_nh.getParam("visualize_obstacles", visualize_obstacles)) {
    ROS_WARN("visualize_obstacles not set (rosparam)");
  }


  vector<double> range_param;
  stringstream ss;
  string param_string;

  private_nh.param("visualization_cube_range", param_string, param_string);
  /* Storing the whole string into string stream */
  ss << param_string;

  /* Running loop till the end of the stream */
  string temp;
  double found;

  while (!ss.eof()) {
    /* extracting word by word from stream */
    ss >> temp;
    /* Checking the given word is integer or not */
    if (stringstream(temp) >> found){
      range_param.push_back(found);
    }
    /* To save from space at the end of string */
    temp = "";
  }

  double xy_width = range_param[0];
  double z_width = range_param[1];
  double stepsize = range_param[2];

  // Data variables
  bool firstTime{true};
  ros::Time start;
  ros::Duration elapsed;
  long iteration{};
  double accumulatedSearchTime1{};
  double accumulatedSearchTime2{};
  double accumulatedSearchTime{};
  double accumulatedTotalTime{};
  double accumulatedHeuristicCost{};
  double accumulatedCostToGoal{};
  double accumulatedTravelledDistance{};
  double accumulatedPrimitiveTime{};

  int accumulatedOpenSet{};
  int accumulatedClosedSet{};
  int accumulatedPlanSize{};
  int accumulatedTrajectorySize{};
  int dynamicObstacleCounter{};

  while (ros::ok()) {

    double time_loop_start = ros::Time::now().toSec();

    std::vector<Obstacle> &obstacles = lattice_planner_node->lattice_planner_.get_dynamic_obstacles();

    
    if(visualize_obstacles){
      lattice_planner_node->publish_visualization_obstacles();
    }

    if(visualize_closed_set)
    {
      lattice_planner_node->publish_visualization_closed_set();
    }

    lattice_planner_node->observe_obstacles(ros::Time::now().toSec());

    auto current_state = lattice_planner_node->dji_state_;
    auto goal_state = lattice_planner_node->goal_state_;
    
    // Reached goal?
    double goal_dist = (current_state.position - goal_state.position).norm();
    ROS_INFO_STREAM_THROTTLE(1, "goal_dist: " << goal_dist << " current_state: " << current_state << " goal_state: " << goal_state);

    summaryfile << elapsed << ", "
                << planning_time << ", " 
                << replanning_time << ", "
                << accumulatedSearchTime1 << ", "   
                << accumulatedSearchTime2 << ", " 
                << accumulatedSearchTime << ", "
                << accumulatedTotalTime << ", "   
                << accumulatedHeuristicCost << ", "   
                << accumulatedCostToGoal << ", "
                << accumulatedOpenSet << ", "   
                << accumulatedClosedSet << ", "   
                << accumulatedPlanSize << ", "
                << accumulatedTrajectorySize << ", "   
                << accumulatedTravelledDistance << ", "   
                << accumulatedPrimitiveTime << ", " 
                << dynamicObstacleCounter << std::endl; 

    if (goal_dist < lattice_planner_node->location_tolerance_)
    {
      //Close the file streams correclty
      logfile.close();
      summaryfile.close();

      //Signal to SetupScenario that we have reached the desired pose for 5 seconds
      ROS_WARN("Planner signaled that it reached goal to SetupScenario, terminating experiment...");
      ros::Time before = ros::Time::now();
      while(ros::Time::now() < before + ros::Duration(5))
      {
        ROS_INFO("Publish reached_goal");
        lattice_planner_node->reached_goal_msg.data = true;
        lattice_planner_node->reached_goal_pub_.publish(lattice_planner_node->reached_goal_msg);
      }
      break;
    }

    if (lattice_planner_node->has_valid_state()) 
    {

      if(firstTime)
      {
        start = ros::Time::now();
        firstTime = false;
      }

      // Planning cycle
      std::pair<bool,bool> result = lattice_planner_node->planning_cycle(planning_time, replanning_time, iteration, visualize_obstacles);

      //--------------- Write data to file -------------------

      elapsed = ros::Time::now() - start;
      double time_to_reach_goal{};
      double heuristic_estimate_to_goal{};
      double cost_from_start_to_goal{};
      double primitive_cost{};
      double primitive_duration{};
      double prev_primitive_cost{};
      double prev_primitive_duration{};
      std::string primitive_name{};

      std::map<std::string, double> marks = lattice_planner_node->get_profiler().get_marks_map();
      double searchTime1 = marks["Graph search 1"];
      double searchTime2 = marks["Graph search 2"];
      double totalSearchTime = searchTime1 + searchTime2;
      double totalTime = lattice_planner_node->get_profiler().total();
      int openSet = lattice_planner_node->lattice_planner_.get_frontier().size();
      int closetSet = lattice_planner_node->lattice_planner_.get_closed_set()->size();
      int planSize = lattice_planner_node->get_current_plan().size();
      int trajPlanSize = lattice_planner_node->get_current_trajectory().size();
      int primitives = lattice_planner_node->lattice_planner_.get_primitives().size();
      
      // Currently not used, instead see collisions.py in SetupScenario/src
      // It does not keep track of if we are already in collision or not.
      int current_collisions{};

      if(!lattice_planner_node->get_current_plan().empty())
      {
        time_to_reach_goal = lattice_planner_node->get_current_plan().back().time;
      }
      
      if(!lattice_planner_node->lattice_planner_.clone_solution_path().empty())
      {
        // .front().cost_f * 2.0 becuase in heuristic_fkn it is * 0.5
        heuristic_estimate_to_goal = (2.0 * lattice_planner_node->lattice_planner_.clone_solution_path().front().cost_f);
        cost_from_start_to_goal = lattice_planner_node->lattice_planner_.clone_solution_path().back().cost_g;    
      }

      double wait_time{};
      if(lattice_planner_node->get_current_plan().size() >= 2)
      {
        //We will always execute the first motion primitive, before we replan
        State next = lattice_planner_node->get_current_plan().at(1);
        int ID = next.actionID;  //Primitive from current state to next_state
        primitive_cost = lattice_planner_node->lattice_planner_.environment.get_primitive(ID).cost;         //Distance [m]
        primitive_duration = lattice_planner_node->lattice_planner_.environment.get_primitive(ID).duration; //Time [s]
        primitive_name = lattice_planner_node->lattice_planner_.environment.get_primitive(ID).name;
        wait_time = lattice_planner_node->lattice_planner_.environment.get_primitive(ID).wait_time;
      }

      accumulatedTravelledDistance = lattice_planner_node->travelled_distance_from_trajectory(lattice_planner_node->get_travelled_path());

      if( (primitive_cost != prev_primitive_cost) && (primitive_duration != prev_primitive_duration) )
      {
        accumulatedPrimitiveTime += primitive_duration;
        prev_primitive_cost = primitive_cost;
        prev_primitive_duration = primitive_duration;
      }

      accumulatedSearchTime1 += searchTime1;
      accumulatedSearchTime2 += searchTime2;
      accumulatedSearchTime += totalSearchTime;
      accumulatedTotalTime += totalTime;
      accumulatedHeuristicCost += heuristic_estimate_to_goal;
      accumulatedCostToGoal += cost_from_start_to_goal;
      accumulatedOpenSet += openSet;
      accumulatedClosedSet += closetSet;
      accumulatedPlanSize += planSize;
      accumulatedTrajectorySize += trajPlanSize;
      dynamicObstacleCounter += current_collisions;


      logfile << ++iteration << ", "                // OK, Number of planning cycles that has been done
              << elapsed << ", "                    // OK , Elapsed simuation time since planning start
              << planning_time << ", "              // OK, planning_time_buffert
              << replanning_time << ", "            // OK, replanning_time_buffert
              << searchTime1 << ", "                // OK, time to complete search1 (primary search)
              << searchTime2 << ", "                // OK, time to complete search2 (secondary search)
              << totalSearchTime << ", "            // OK, search1 + search2
              << totalTime << ", "                  // OK, planning cycle time, includes memory cleanup, preperations, search1 and search2
              << heuristic_estimate_to_goal << ", " // OK, heuristic esstimate to goal
              << cost_from_start_to_goal << ", "    // OK, Cost from start state to goal_state
              << openSet << ", "                    // OK, Size of openset
              << closetSet << ", "                  // OK, Size of closedset
              << planSize << ", "                   // OK, number of states in current_plan
              << trajPlanSize << ", "               // OK, numnber of trajectory states in current trajectory
              << time_to_reach_goal << ", "         // OK, summing up all primtive duration in plan. NOTE state.actionID gives the primitive that leads TO this state from a parent state.
              << primitive_cost << ", "             // primt
              << primitive_duration << ", " 
              << current_collisions << ", "
              << result.first << ", "
              << result.second << ", "
              << primitive_name << ", "
              << primitives << std::endl;   
        
      // -----------------------------------------------------

      lattice_planner_node->new_nav_goal_ = false;
      double loop_time = ros::Time::now().toSec() - time_loop_start;
    } 
    else 
    {
    ROS_WARN_STREAM_THROTTLE(1, "Missing valid pose, velocity or goal");
    }

    if (CountOpendMap++ % 7 == 0)
    {
      if (enable_visualization_close_proximity) lattice_planner_node->publish_visualization_octomap(xy_width,z_width,stepsize);
      if (enable_visualization_grid) lattice_planner_node->publish_visualization_octomap_plane_grid();
    }

    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
  }

  ros::shutdown();
  return 0;
}


// ---------------------------------- CLASS MEMBERS DEFINITION -------------------------------------

double LatticePlannerNode::travelled_distance_from_trajectory(visualization_msgs::Marker line_list)
{

  double dist{};

  if(line_list.points.empty() || line_list.points.size() == 1)
  {
    return dist;
  }

  for(int i = 0; i < line_list.points.size()-1; i++)
  {
    Eigen::Vector3d p1 (line_list.points[i].x, line_list.points[i].y, line_list.points[i].z);
    Eigen::Vector3d p2 (line_list.points[i+1].x, line_list.points[i+1].y, line_list.points[i+1].z);
    Eigen::Vector3d dir = p2 - p1;
    dist += dir.norm();
  }

  return dist;
}


bool LatticePlannerNode::initialize(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
  ROS_INFO("Initializing LatticePlannerNode");
  nh_ = nh;
  private_nh_ = private_nh;

  dji_state_ = SearchState();
  new_nav_goal_ = false;
  previous_cycle_time_ = ros::Time::now();

  std::string primitive_path;
  if (!private_nh.getParam("primitive_path", primitive_path)) {
    ROS_ERROR("primitive_path not set (rosparam)");
    abort();
  }

  int number_of_primitive_groups;
  if (!private_nh.getParam("number_of_primitive_groups",
                           number_of_primitive_groups)) {
    ROS_ERROR("number_of_primitive_groups not set (rosparam)");
    abort();
  }

  // The planner waits until a vaild pose and vel message have been sent
  this->has_valid_pose_ = false;
  this->has_valid_vel_ = false;
  this->has_valid_goal_ = false;
  this->has_valid_scenario_ = false;
  lattice_planner_node->set_valid_obstacles(false);
  this->last_state_receeding_horizon = false;
  this->last_state_emergency_avoidance = false;

  this->grid_size_ = 0.50; // OBS: Must correspond to the primitives loaded by lattice_planner_

  vector<int> primitive_amount_in_group;

  for (int i = 0; i < number_of_primitive_groups; i++) {

    int primitive_amount;
    string param_name = string("primitive_amount_group_") + to_string(i);

    if (!private_nh.getParam(param_name.c_str(), primitive_amount)) {
      ROS_ERROR_STREAM("primitive_amount_group_" << i << " not set (rosparam)");
      abort();
    } else {
      primitive_amount_in_group.push_back(primitive_amount);
    }
  }

  double uav_safety_radius = 1.0;

  if (!private_nh.getParam("dji_safety_radius", uav_safety_radius)) {
    ROS_WARN("dji_safety_radius not set (rosparam)");
    abort();
  }

  bool use_stand_still = true;
  bool use_geometric_secondary_search = true;
  bool use_only_geometric_search = false;
  double plan_duration_minimum = 5.0;
  std::string closed_set_type = "SetWaitTime";
  bool best_effort = true;
  bool visualize_frontiers{};

  if (!private_nh.getParam("adaptive_planning", this->adaptive_planning_)) {
    ROS_WARN("adaptive_planning not set (rosparam)");
  }

  if(this->adaptive_planning_)
  {
    ROS_INFO("Adaptive planning is [ACTIVE]");
  }
  else
  {
    ROS_INFO("Adaptive planning is [INACTIVE]");
  }

  if (!private_nh.getParam("survival_planning", this->survival_planning_)) 
  {
    ROS_ERROR("survival_planning not set (rosparam)");
  }

  if(this->survival_planning_)
  {
    ROS_INFO("Survival planning is [ACTIVE]");
  }
  else
  {
    ROS_INFO("Survival planning is [INACTIVE]");
  }


  if (!private_nh.getParam("emergency_trajectories", this->emergency_trajectories_)) {
    ROS_WARN("emergency_trajectories not set (rosparam)");
  }

  if(this->emergency_trajectories_)
  {
    ROS_INFO("Emergency trajectories is [ACTIVE]");
  }
  else
  {
    ROS_INFO("Emergency trajectories is [INACTIVE]");
  }

  if (!private_nh.getParam("use_stand_still", use_stand_still)) {
    ROS_WARN("use_stand_still not set (rosparam)");
  }
  if (!private_nh.getParam("use_geometric_secondary_search",
                           use_geometric_secondary_search)) {
    ROS_WARN("use_geometric_secondary_search not set (rosparam)");
  }
  if (!private_nh.getParam("use_only_geometric_search",
                           use_only_geometric_search)) {
    ROS_WARN("use_only_geometric_search not set (rosparam)");
  }
  if (!private_nh.getParam("plan_duration_minimum", plan_duration_minimum)) {
    ROS_WARN("plan_duration_minimum not set (rosparam)");
  }
  if (!private_nh.getParam("closed_set_type", closed_set_type)) {
    ROS_WARN("closed_set_type not set (rosparam)");
  }
  if (!private_nh.getParam("best_effort", best_effort)) {
    ROS_WARN("best_effort not set (rosparam)");
  }
  if (!private_nh.getParam("use_predictions", this->use_predictions_)) {
    ROS_WARN_STREAM("Using obstacles predicted motions as default");
  }
  if (!private_nh.getParam("visualize_frontiers", visualize_frontiers)) {
    ROS_WARN("visualize_frontiers not set (rosparam)");
  }

  // Initialize lattice planner
  bool success = lattice_planner_.initialize(primitive_path, 
                                            primitive_amount_in_group, 
                                            uav_safety_radius, 
                                            &profiler, 
                                            nh,
                                            use_stand_still, 
                                            use_geometric_secondary_search,
                                            use_only_geometric_search, 
                                            plan_duration_minimum, 
                                            closed_set_type,
                                            best_effort, 
                                            true, 
                                            visualize_frontiers, 
                                            emergency_trajectories_, 
                                            this->survival_planning_);

  LOG("option_use_stand_still", 0.0, use_stand_still);
  LOG("option_use_geometric_secondary_search", 0.0,
      use_geometric_secondary_search);
  LOG("option_use_only_geometric_search", 0.0, use_only_geometric_search);
  LOG("option_plan_duration_minimum", 0.0, plan_duration_minimum);
  LOG("option_best_effort", 0.0, best_effort);
  LOG("option_uav_safety_radius", 0.0, uav_safety_radius);
  LOG("option_grid_size", 0.0, this->grid_size_);

  if (!success) {
    ROS_ERROR("Initialization of lattice planner failed");
    return false;
  }

  // ----------------------------------------- Define subscribers -----------------------------------------
  ROS_INFO("Initializing subscribers");
  pose_sub_ = nh.subscribe<geometry_msgs::PointStamped>("dji_sdk/local_position", 1, &LatticePlannerNode::poseCallback, this);
  vel_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 1, &LatticePlannerNode::velCallback, this);
  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("nav_goal", 1, &LatticePlannerNode::goalCallback, this);
  obstacles_sub_ = nh.subscribe<lattice_planner::Obstacles>("obstacles", 1, &LatticePlannerNode::obstacleCallback, this);
  scenario_sub_ = nh.subscribe<lattice_planner::Scenario>("scenario_info", 1, &LatticePlannerNode::scenarioCallback, this);
  oct_sub_ = nh.subscribe<octomap_msgs::Octomap>("octomap_full", 1, &LatticePlannerNode::octomapCallback, this);

  // ----------------------------------------- Define publishers -----------------------------------------
  ROS_INFO("Initializing publisher");
  trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
  trajectory_id_pub_ = nh.advertise<lattice_planner::plantime>("plantime", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
  reached_goal_pub_ = nh.advertise<std_msgs::Bool>("/dji0/reached_goal_planner", 1);

  //Visualization
  plan_trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_plan_trajectory", 1);
  plan_trajectory_safety_radius_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_plan_trajectory_safety_radius", 1);
  dji_hitbox_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_vehicle", 1);
  plan_obstacles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_obstacles_lp", 1);
  obstacle_plan_trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_obstacles_plan", 1);
  travelled_path_pub_ = nh.advertise<visualization_msgs::Marker>("ground_truth/travelled_path",10);
  closed_set_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_closed_set", 1);
  octomap_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_openmap", 1);
  octomap_plane_occupied_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("visualization_occupied_grid", 1);
  anchor_point_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_anchor_point", 1);
  end_point_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_end_point", 1);
  debug_pose_pub_ = nh.advertise<visualization_msgs::Marker>("debug_pose", 1);

  //Setting params for travelled path
  lattice_planner_node->line_list_.header.frame_id = "world";
  lattice_planner_node->line_list_.header.stamp = ros::Time::now();
  lattice_planner_node->line_list_.action = visualization_msgs::Marker::ADD;
  lattice_planner_node->line_list_.pose.orientation.w = 1.0;
  lattice_planner_node->line_list_.id = 0;
  lattice_planner_node->line_list_.type = visualization_msgs::Marker::LINE_STRIP;
  lattice_planner_node->line_list_.scale.x = 0.1;
  lattice_planner_node->line_list_.color.r = 1.0;
  lattice_planner_node->line_list_.color.a = 1.0;

  //Inititally the goal has not been reached
  lattice_planner_node->reached_goal_msg.data = false;

  object_lattice_planner_.initialize(primitive_path,
                                     primitive_amount_in_group,
                                     uav_safety_radius,
                                     &object_profiler,
                                     nh,
                                     false,
                                     false,
                                     true,
                                     0.0,
                                     "Set",
                                     false, 
                                     false,
                                     false,
                                     false,
                                     false);

  ROS_INFO("Initialization done");

  return true;
}


void LatticePlannerNode::add_obstacle(Obstacle o, bool dynamic = true) 
{
  lattice_planner_.add_obstacle(o);
  if (!dynamic) {
    this->num_static_obstacles++;
  }
  ROS_INFO_STREAM("Added Obstacle: name=" << o.get_name() << " x= " << o.get_hitbox().getX() << " y= " << o.get_hitbox().getY() << " z= " << o.get_hitbox().getZ() << " radius= " << o.get_hitbox().getR());
}

/*
This function performs the Lattice Planner planning cycle, it consits of a replan step and a plan step. The planning step is done to find an 
initial plan to the goal, and the replan step is done to find a new plan if the current plan is no longer valid. The replanning step is very
important as it ensures that the UAV can avoid obstacles and reach the goal safely. The function returns a pair of bools, where the first bool
indicates if a new plan was found, and the second bool indicates if the same plan as the previous was used.
*/
std::pair<bool, bool> LatticePlannerNode::planning_cycle(double planning_time, double replanning_time, int iteration = 0, bool visualize_obstacles) 
{
  ROS_INFO_STREAM("-------------------------- Entering Planning Cycle | Iteration << " << iteration << " >> --------------------------");
  
  // Variables
  bool replanning_enabled = true;
  double cost_to_change_plan = 10.0;               // Used by re-planning
 
  // Planner variables
  bool plan_found = false;
  bool plan_new = false;
  bool use_old_plan = false;
  double time_since_last_cycle{};
  double planning_duration{};
  vector<TrajectoryState> new_trajectory;
  SearchState closestSearchState;
  bool success = false;


  if (new_nav_goal_) 
  {
    ROS_INFO("New navigation goal aquired in Lattice Planner");
  }

  if (is_plan_in_the_past(this->current_plan_)) 
  {
    ROS_WARN("Current plan is in the past (Timestamps in plan is outdated)");
    this->current_plan_.clear();
    this->current_trajectory_.clear();
  }

  // Start timer for planning cycle
  double time_of_plan = ros::Time::now().toSec();

  // ########################### REPLAN STEP #################################
  // #########################################################################

  if (!this->current_trajectory_.empty() && replanning_enabled)
  {
    ROS_INFO("Plan exists and replanning enabled -> Trying to replan...");
    // We can only replan (reconstruct current plan) if there exists a plan
    // Also replanning must be enabled.

    time_since_last_cycle = time_of_plan - this->previous_cycle_time_.toSec();
    ROS_INFO_STREAM("Time since last cycle: " << time_since_last_cycle);

    time_adjust_plan(this->current_plan_, this->current_trajectory_, time_since_last_cycle);
  
    ROS_INFO("REPLAN: Searching for secondary plan start index...");
    int secondary_plan_start_index = calculate_secondary_plan_index(this->current_plan_);

    bool current_plan_collision_free = false;
    int primary_plan_end_index = current_plan_.size() - 1;

    if (secondary_plan_start_index != this->current_plan_.size()) 
    {
      ROS_INFO("REPLAN: Secondary Search was used!");
      int secondary_trajectory_index = calculate_secondary_trajectory_start_index(this->current_trajectory_, 
                                                                                  this->current_plan_,
                                                                                  secondary_plan_start_index);

      primary_plan_end_index = std::min((int)this->current_plan_.size() - 1, std::max(0, secondary_plan_start_index - 1));

      std::tuple<bool, double, TrajectoryState> result = plan_in_collision(this->current_trajectory_, 0.0, secondary_trajectory_index);
      current_plan_collision_free = !std::get<0>(result);
      time_until_collision_ = std::get<1>(result);
      collision_state_ = std::get<2>(result);
    } 
    else 
    {
      ROS_INFO("REPLAN: Primary Search was used - secondary plan disregarded");
      std::tuple<bool, double, TrajectoryState> result = plan_in_collision(this->current_trajectory_, 0.0);
      current_plan_collision_free = !std::get<0>(result);
      time_until_collision_ = std::get<1>(result);
      collision_state_ = std::get<2>(result);
    }

    // The time duration of the current PRIMARY plan is larger or equal than the minimum allowed duration
    bool current_plan_duration_valid = this->current_plan_[primary_plan_end_index].time >= lattice_planner_.get_plan_duration_minimum();
        
    if (lattice_planner_.use_only_geometric_search)
    {
      current_plan_duration_valid = true;
    }

    // Check if the plan is leading to the TRUE goal
    bool current_plan_reach_goal = is_plan_leading_to_goal(this->current_plan_, lattice_planner_.goalState);
    
    // Aggregate bools to see if entire plan is valid
    bool current_plan_valid = current_plan_collision_free && current_plan_duration_valid && current_plan_reach_goal;

    if (current_plan_valid) 
    {
      ROS_INFO("REPLAN: Current plan is valid (collision free, fulfills plan duration requirement and leads to the goal)");
    } 
    else 
    {
      if (!current_plan_collision_free) 
      {
        ROS_WARN_STREAM("REPLAN: Current plan in collision with collision in: " << time_until_collision_ << " seconds!");
        
        /*
        SearchState collision_search_state {collision_state_.position.x(), 
                                          collision_state_.position.y(), 
                                          collision_state_.position.z(), 
                                          collision_state_.velocity.x(), 
                                          collision_state_.velocity.y(), 
                                          collision_state_.velocity.z()};
        */


        //debug_pose(collision_search_state);      
      }

      if (!current_plan_duration_valid) 
      {
        ROS_WARN("REPLAN: Current plan is no longer valid due to having too short duration!");
        ROS_WARN_STREAM("REPLAN: Current duration: " << this->current_plan_[primary_plan_end_index].time << " Minimum duration: " << lattice_planner_.get_plan_duration_minimum());
      }

      if(!current_plan_reach_goal)
      {
        ROS_WARN("REPLAN: Current plan is not valid due to it not leading to the true goal, but a temporary one!");
      }
    }

    // IMPROVEMENT 1: Pick suitable index and replanning_time
    // Find suitable grid point to start searching from (the next on the previous path)
    // given minimum duration and time_until_collision_
    // Duration is how far into time we start the replanning process

    //Default implemenation
    double prefix_duration = 3 * (replanning_time * (1.0/3.0)) + 0.1; //From old lattice_planner
    int index{};

    if(!this->adaptive_planning_)
    {
      if (current_plan_duration_valid)
      {
        index = next_path_state(prefix_duration, this->current_plan_);
      }
      else if (!current_plan_collision_free)
      {
        index = next_path_state(prefix_duration, this->current_plan_);
      }
      else if (!current_plan_duration_valid)
      {
        index = next_path_state(std::min(prefix_duration, (double)std::max(this->current_plan_[primary_plan_end_index].time, 0.0)), this->current_plan_);
      }
    }
    else
    {
      //Adaptive replanning
      if (current_plan_collision_free && current_plan_duration_valid)
      {
        ROS_WARN("Collision and Duration OK: Index := min_duration * 2");
        //index = next_path_state(this->current_plan_[primary_plan_end_index].time, this->current_plan_);
        index = next_path_state(lattice_planner_.get_plan_duration_minimum() * 2, this->current_plan_);
      }
      else if (current_plan_duration_valid && (time_until_collision_ > lattice_planner_.get_plan_duration_minimum()))
      {
        ROS_WARN("Duration OK and time_until_collision > min_duration: Index := min_duration");
        index = next_path_state(lattice_planner_.get_plan_duration_minimum(), this->current_plan_);
      }
      else if (!current_plan_collision_free)
      {
        ROS_WARN("Collision within duration: Index = time_until_collision * 2/3");
        index = next_path_state(time_until_collision_ * (2.0/3.0), this->current_plan_);
      }
      else if (!current_plan_duration_valid)
      {
        ROS_WARN("Duration not OK: Index := min(min_dur, max(end_index_time, 0))");
        index = next_path_state(std::min(lattice_planner_.get_plan_duration_minimum(), (double)std::max(this->current_plan_[primary_plan_end_index].time, 0.0)), this->current_plan_);
      }
      else
      {
        ROS_ERROR_STREAM("REPLAN: No suitable index found for replanning! current_plan_duration_valid: " << current_plan_duration_valid << " current_plan_collision_free: " << current_plan_collision_free);
      }
    }

    if (index < this->current_plan_.size()) 
    {
      ROS_INFO_STREAM("REPLAN: Next state to replan from has index: " << index << " and state: " << this->current_plan_[index]);
    } 
    else 
    {
      ROS_ERROR_STREAM("REPLAN: next_path_state index is out of bounds! index= " << index << " but this->current_plan_.size()= " << this->current_plan_.size());
    }
   
    // ----------- START Replanning -----------

    // We do a replan even if current_plan_valid == true. This is
    // since we might find a cheaper/better plan than the previous plan!
    // We send in current_plan_valid so that we can check if we 
    // have a valid plan before we do find_best_destination

    double time_before_planning = time_of_plan;
    double start_time = this->current_plan_[index].time;
    
    if(this->previous_iteration_was_emergency)
    {

      //This part is not part of the planning process, thus stop the timer here.      
      this->previous_cycle_time_ = ros::Time::now();
      
      //Previous iteration was an emergency and the dji are no longer on the lattice, do replan from closest state.
      ROS_ERROR("Find closest point on lattice!");
      closestSearchState = round_search_state_and_collision_check(this->current_plan_.back().state);

      //Wait for dji to reach final point on emergency trajectory.   
      double distance = (this->dji_state_.position - this->current_plan_.back().state.position).norm();

      // Visualize end pose (debug)
      debug_pose(this->current_plan_.back().state);

      while(distance > 0.05)
      {
        //Wait to reach final pose on emergency...
        ros::spinOnce();
        distance = (this->dji_state_.position - this->current_plan_.back().state.position).norm();
      }

      ROS_ERROR("Dji reached final point on emergency trajectory!");

      //Re-align with Lattice
      this->current_plan_.clear();
      this->current_trajectory_.clear();
      generate_global_trajectory(); 
      ros::spinOnce(); // Spin once to update this->dji_state_

      ROS_ERROR("Realign with lattice!");
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = closestSearchState.position.x();
      msg.pose.position.y = closestSearchState.position.y();
      msg.pose.position.z = closestSearchState.position.z();
      msg.pose.orientation.w = 1.0;
      pose_pub_.publish(msg);

      //After this point the dji should be on the lattice again
      //also the previous plan and trajectory cannot be conneced to since they are off lattice
      success = true;
      this->previous_iteration_was_emergency = false;
      return std::make_pair(success, use_old_plan);

    }
    else
    {
      //Previous iteration was not an emergency, proceed as normal
      closestSearchState = (this->current_plan_[index].state);
    }


    // IMPROVEMENT 3: Emergency Trajectories
    //if(this->emergency_trajectories_ && (time_until_collision_ < start_time) && (time_until_collision_ < lattice_planner_.get_plan_duration_minimum()))
    if(this->emergency_trajectories_ && (time_until_collision_ < this->current_plan_.front().time))
    {
      ROS_ERROR_STREAM("Emergency Trajectories: Time until collision: " << time_until_collision_ << " seconds < " << this->current_plan_.front().time << " seconds");

      if(this->current_trajectory_.empty())
      {
        ROS_ERROR("REPLAN: Current trajectory is empty, cannot find anchor point!");
        this->previous_cycle_time_ = ros::Time::now();
        return std::make_pair(success, use_old_plan);
      }
      
      // Step 1: Extract first anchor point or just before collision
      // We could also extract a set of candidates, from the first one till collision 
      int anchor_point_index{};
      anchor_point_index = next_trajectory_state((time_until_collision_ * (1.0/2.0)), this->current_trajectory_); //Tuning parameter
      TrajectoryState anchor_point = this->current_trajectory_[anchor_point_index];

      // Step 2: Generate emergency trajectory (Currently not pre-computed)

      // Transform anchor_point to SearchState
      SearchState anchor_point_ss {anchor_point.position.x(), 
                                   anchor_point.position.y(), 
                                   anchor_point.position.z(), 
                                   anchor_point.velocity.x(), 
                                   anchor_point.velocity.y(), 
                                   anchor_point.velocity.z()};
      
      this->anchor_point = anchor_point;

      //Visualize anchor point
      publish_anchor_point(anchor_point_ss); 

      // Send to expand_frontier_from_state and get possible MPs, if we can expand, modify new_trajectory
      replanning_time = time_until_collision_ * (1.0/4.0); //Tuning paramter
      plan_found = lattice_planner_.expand_emergency_from_state(anchor_point_ss, this->goal_state_, replanning_time + 0.4, new_trajectory, anchor_point.time + 0.4, collision_state_, time_until_collision_, current_plan_valid);
      
      ROS_INFO_STREAM("Emergency found: " << std::boolalpha << plan_found);

      if(plan_found)
      {
      
      // Time adjust and set new plan and trajectory
      planning_duration = ros::Time::now().toSec() - time_before_planning;

      std::vector<State> new_plan = lattice_planner_.clone_solution_path();
      
      time_adjust_plan(new_plan, new_trajectory, planning_duration);

      // Visualize end point on emergency trajectory
      publish_off_lattice_pos(new_plan.back().state);

      ROS_INFO("Emergency: Attaching Emergency to old plan!");

      // Calculate trajectory prefix using time duration since last plan was published
      std::vector<TrajectoryState> total_trajectory;
      std::vector<TrajectoryState> trajectory_prefix = calculate_trajectory_prefix(0.0, new_plan[0], this->current_trajectory_);
      
      total_trajectory.insert(total_trajectory.end(), trajectory_prefix.begin(), trajectory_prefix.end());
      total_trajectory.insert(total_trajectory.end(), new_trajectory.begin(), new_trajectory.end());
     
      this->current_trajectory_ = total_trajectory;
      this->current_plan_ = new_plan;
      this->previous_iteration_was_emergency = true;

      }
      else
      {
        ROS_WARN_STREAM("Emergency Trajectories: No emergency trajectory found!");
      }
    }
    else
    {
      if(this->adaptive_planning_)
      {
        if(start_time <= 0.1)
        {
          replanning_time = start_time;
        }
        else
        {
          replanning_time = start_time - 0.1; //Sometimes we get a very small negative number in new_plan.front(). this constant is to compensate for that so it never happens.
        }
        ROS_ERROR_STREAM("Adaptive replanning time: " << replanning_time);
      
        if(replanning_time > 100)
        {
          abort();
        }
      }

      plan_found = lattice_planner_.do_planning_cycle(closestSearchState, this->goal_state_, replanning_time + 0.4, new_trajectory, start_time + 0.4, collision_state_, time_until_collision_, current_plan_valid);

      // ----------- END Replanning -----------
      planning_duration = ros::Time::now().toSec() - time_before_planning;

      std::vector<State> new_plan = lattice_planner_.clone_solution_path();
      
      ROS_INFO_STREAM("REPLAN: Replanning took: " << planning_duration << " seconds! Max time budget: " << replanning_time << " second(s)");

      // Time adjust current plan with planning duration
      time_adjust_plan(this->current_plan_, this->current_trajectory_, planning_duration);
      
      //Returns the first index in current_plan_
      index = next_path_state(0.0, this->current_plan_);
      
      // Time adjust new plan with planning duration
      time_adjust_plan(new_plan, new_trajectory, planning_duration);

      bool found_new_plan = plan_found && this->is_new_plan(new_plan, this->current_plan_, index);
      
      if (found_new_plan)
      {
        ROS_INFO_STREAM("REPLAN: Found new plan after re-planning");
      }

      // Keep old plan if it is sill valid and not much more expensive than the
      // new plan. This is to avoid behaviour where the planner switches rapidly 
      // between to plans that are essentially the same.

      if (plan_found && current_plan_valid && !new_nav_goal_)
      {
        double cost_old = calculate_plan_cost(this->current_plan_, 0.0);
        double cost_new = new_plan.back().cost_g;
        
        ROS_INFO_STREAM("REPLAN: Cost old plan: " << cost_old << " Cost new plan: " << cost_new);

        if (found_new_plan && cost_old <= cost_new + cost_to_change_plan) 
        {
          ROS_INFO("REPLAN: Found new plan, but old plan has lower cost or it's to expensive to change!");
          use_old_plan = true;
        }
        else
        {
          if(!found_new_plan)
          {
            ROS_INFO("REPLAN: Found no plan or new plan is the same as the previous one!");
            // use_old_plan remains false -> We will reconnect with new plan
          }

          if(!(cost_old <= cost_new + cost_to_change_plan))
          {
            ROS_INFO("REPLAN: New plan is cheaper than old plan, prepare to reconnect!");
            // use_old_plan remains false -> We will reconnect with new plan
          }
        }
      } 
    
      // Keep old plan if it is still valid and no new plan was found 
      // OR if the new plan is the same as the old one
      if (!found_new_plan && current_plan_valid && !new_nav_goal_) 
      {
        use_old_plan = true;
        if (plan_found && !found_new_plan) 
        {
          ROS_INFO("REPLAN: Found new plan, but same as old plan!");
        } 
        else 
        {
          ROS_INFO("REPLAN: Found no new plan found, but the old one is still valid!");
        }
      }

      // Use new plan with appended prefix
      // 1. If we found a new plan that is not the same as the old one and we should not use the old plan
      // 2. If the current plan is no longer valid (collision-free, within time duration or leads to goal)
      // but we found a new plan that is.
      if ( (found_new_plan && !use_old_plan) || (!current_plan_valid && plan_found) ) 
      {        
        if (new_plan.front().time < -0.01) 
        { 
          // From Emil et al
          /*
          So when we active find_best_destination, there is a possibility that when we do replan, we do it from at state
          that is very close to the drone. If we then plan and come up with plan from the next state, but at the same time
          pass that very state since the NMPC works on its own, the plan will be in the past and no longer valid.

          Ways of handling this:
          1. Set the replanning time to the time before we reach the intended state the diverge from, meaning that we
          need to have a plan at latest when we reach that state. This is what happens now and it works quite well.
          However if collision is within the time of reaching the first state, we need emergency trajectories.
          */

          ROS_FATAL("REPLAN: Appending prefix to new plan is not possible, new plan has diverged from current plan. ");
          ROS_FATAL("WE SHOULD NEVER END UP HERE!");
          this->previous_cycle_time_ = ros::Time::now();
          success = false;
          return std::make_pair(success, use_old_plan);
        }

        ROS_INFO("REPLAN: Attaching new plan to old plan!");

        // Calculate trajectory prefix using time duration since last plan was published
        std::vector<TrajectoryState> total_trajectory;
        std::vector<TrajectoryState> trajectory_prefix;
        if(this->previous_iteration_was_emergency)
        {
          ROS_INFO_STREAM("REPLAN: Previous iteration was an emergency, using anchor point as startState!");
          trajectory_prefix = calculate_trajectory_prefix2(0.0, new_trajectory[0], this->current_trajectory_);
        }
        else
        {
          trajectory_prefix = calculate_trajectory_prefix(0.0, new_plan[0], this->current_trajectory_);
        }
        
        total_trajectory.insert(total_trajectory.end(), trajectory_prefix.begin(), trajectory_prefix.end());
        total_trajectory.insert(total_trajectory.end(), new_trajectory.begin(), new_trajectory.end());
        
        this->current_trajectory_ = total_trajectory;
        this->current_plan_ = new_plan;
      }
      else
      {
        ROS_INFO("REPLAN: No NEW plan found, resorting to old plan...");    
        /**
         * Ending up here means that
         *  1. We have not found a NEW plan in the sense that it differs from the old on, but we can still use the old one.
         *  2. The current plan is still valid and we have not found a plan in the planning_cycle.
         *  So we will continue to use the old plan until the next planning cycle.
         */
      
      }
    this->previous_iteration_was_emergency = false;
    }
  } 
  else 
  {
    // ########################### PLANNING STEP #################################
    // ###########################################################################
   
    ROS_INFO("PLAN: Creating new plan (Planning) ...");
    
    //Cleanup
    this->current_plan_.clear();
    this->current_trajectory_.clear();

    //Search for new plan, search from a point on lattice closest to dji.
    closestSearchState = round_search_state(this->dji_state_);
    plan_found = lattice_planner_.do_planning_cycle(closestSearchState, this->goal_state_, planning_time, new_trajectory, 0.4, collision_state_, time_until_collision_);
    
    //Set new plan and trajectory
    this->current_trajectory_ = new_trajectory;
    this->current_plan_ = lattice_planner_.clone_solution_path();
  }
  
  if (use_old_plan) 
  {
    ROS_INFO("GENERAL: Using plan from previous planning iteration!");
    success = true;
  } 
  else if (plan_found) 
  {
    ROS_INFO("GENERAL: New Plan was found to goal!");
    //publish_visualization_plan_trajectory(Dji_reference_trajectory_,-1);
    
    // Construct MultiDOFJointTrajectory and plantime object from current_trajectory_ and current_plan_
    generate_global_trajectory();
    generate_global_plan();
  
    Dji_reference_trajectory_.header.stamp = ros::Time::now();
    Dji_reference_plan_.header.stamp = ros::Time::now();
    
    // Send plan to NMPC for execution
    trajectory_pub_.publish(Dji_reference_trajectory_);
    trajectory_id_pub_.publish(Dji_reference_plan_);
    success = true;
  } 
  else 
  {
    ROS_FATAL("GENERAL: No feasible plan found AND cannot use old plan, executing stop command!");
    stop_command();
    success = false;
  }

  // ------------------ Visualization -----------------

  int secondary_plan_start_index = calculate_secondary_plan_index(this->current_plan_);
  bool secondary_plan_exists = secondary_plan_start_index != this->current_plan_.size();
  
  if (secondary_plan_exists) 
  {
    // The plan is partly computed from the secondary planning phase
    int secondary_trajectory_index = calculate_secondary_trajectory_start_index(this->current_trajectory_, this->current_plan_, secondary_plan_start_index);
    publish_visualization_plan_trajectory(Dji_reference_trajectory_, secondary_trajectory_index);
    publish_visualization_plan_trajectory_safety_radius(Dji_reference_trajectory_, secondary_trajectory_index);
  } 
  else 
  {
    // The complete plan in from the primary planning phase
    publish_visualization_plan_trajectory(Dji_reference_trajectory_, -1);
    publish_visualization_plan_trajectory_safety_radius(Dji_reference_trajectory_, -1);
  }

  publish_visualization_travelled_path();
  
  if(visualize_obstacles)
  {
    publish_visualization_obstacles(lattice_planner_.get_obstacles());
    publish_visualization_obstacle_plan_trajectory(lattice_planner_.get_obstacles());
  }

  this->previous_cycle_time_ = ros::Time::now();
  return std::make_pair(success, use_old_plan);
}


/*
Send an empty plan to the MPC controller, forcing it 
to default to the last command/pose. Used as a final resort if not plan
is found, currently it will force the DJI to remain where it is!
*/
void LatticePlannerNode::stop_command() 
{
  this->current_plan_.clear();
  this->current_trajectory_.clear();
  generate_global_trajectory(); 
  ros::spinOnce(); // Spin once to update this->dji_state_

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = this->dji_state_.position.x();
  msg.pose.position.y = this->dji_state_.position.y();
  msg.pose.position.z = this->dji_state_.position.z();
  msg.pose.orientation.w = 1.0;
  pose_pub_.publish(msg);
}


/*
Given a new_plan and a previous plan and a starting index, will
this function return true if the new_plan is equal to the 
previous plan starting from the start_index, otherwise false.
*/
bool LatticePlannerNode::is_new_plan(std::vector<State> new_plan,
                                     std::vector<State> prev_plan,
                                     int start_index) {

  // If not same size with adj start_index, the plan is new
  if (new_plan.size() + start_index - prev_plan.size() != 0)
    return true;

  // Something is wrong!
  if (new_plan.size() == 0 || prev_plan.size() == 0) {
    ROS_WARN("IS_NEW_PLAN: new_plan.size() is 0 and prev_plan.size() is 0!");
    return true;
  }

  for (int n = 0; n < new_plan.size(); n++) {
    if (new_plan[n].state != prev_plan[n + start_index].state)
      return true;
  }

  return false;
}


/*
Given a time duration and a plan, will this function return the INDEX
to the element in the plan which is *within* the given time duration.
If time_duration == 0.0, it returns the index to first element.
*/
int LatticePlannerNode::next_path_state(double time_duration, std::vector<State> &plan) 
{
  if (plan.empty() || (time_duration == 0.0))
    return 0;
  int index = 0;
  while (index < plan.size() - 1 && plan[index].time <= time_duration) {
    index++;
  }
  return index;
}


/*
Given a time duration and a trajectory, will this function return the INDEX
to the element in the plan which is *within* the given time duration.
If time_duration == 0.0, it returns the first element*/
int LatticePlannerNode::next_trajectory_state(double time_duration, std::vector<TrajectoryState> &trajectory) 
{
  if (trajectory.empty() || (time_duration == 0.0))
    return 0;
  int index = 0;
  while (index < trajectory.size() - 1 && trajectory[index].time <= time_duration) {
    index++;
  }
  return index;
}


/*
Given a plan and trajectory, will this function adjust each element so that
they are up to date time-wise, since time has passed since the last planning cycle. 
*/
void LatticePlannerNode::time_adjust_plan(std::vector<State> &plan, std::vector<TrajectoryState> &trajectory, double dt) 
{
  for (int n = 0; n < plan.size(); n++)
    plan[n].time -= dt;
  for (int n = 0; n < trajectory.size(); n++)
    trajectory[n].time -= dt;
}


/*
This function will return projected distance between two states.
*/
double projected_distance(SearchState &s1, SearchState &s2, SearchState &p) 
{
  if ((s1.position - s2.position).squaredNorm() < 1e-3)
    return 0.0;
  SearchState state = s1;
  state.position -= s2.position;
  state.position.normalize();
  return state.position.dot(p.position - s1.position);
}


/*
Helper function that will return the projected distance between two states.
*/
double projected_distance(State *s1, State *s2, SearchState &p) 
{
  return projected_distance(s1->state, s2->state, p);
}


/*
This function will return the index of the state in the path that is closest to the given state spatial-wise.
*/
int find_closest_path_state(SearchState state, const std::vector<State *> &path) 
{

  double min_distance = 10e10;
  int min_index = 0;
  double distance;
  for (int n = 0; n < path.size(); n++) {
    distance = (state.position - path[n]->state.position).norm();
    if (distance < min_distance) {
      min_distance = distance;
      min_index = n;
    }
  }
  return min_index;
}


/*
This returns the cloest state on the lattice given a SearchState.
*/
SearchState LatticePlannerNode::round_search_state(SearchState state) 
{

  SearchState roundedState(state);

  roundedState.position.x() = round(state.position.x() / grid_size_) * grid_size_;
  roundedState.position.y() = round(state.position.y() / grid_size_) * grid_size_;
  roundedState.position.z() = round(state.position.z() / grid_size_) * grid_size_;
  roundedState.velocity.x() = 0.0 * round(state.velocity.x());
  roundedState.velocity.y() = 0.0 * round(state.velocity.y());
  roundedState.velocity.z() = 0.0 * round(state.velocity.z());

  return roundedState;
}


/*
This returns the closest state on the lattice and collision checks it.
*/
SearchState LatticePlannerNode::round_search_state_and_collision_check(SearchState state) 
{

  SearchState roundedState(state);
  roundedState.position.x() = round(state.position.x() / grid_size_) * grid_size_;
  roundedState.position.y() = round(state.position.y() / grid_size_) * grid_size_;
  roundedState.position.z() = round(state.position.z() / grid_size_) * grid_size_;
  roundedState.velocity.x() = round(state.velocity.x());
  roundedState.velocity.y() = round(state.velocity.y());
  roundedState.velocity.z() = round(state.velocity.z());

  State tempState = State(roundedState);

  if(this->lattice_planner_.environment.in_collision(lattice_planner_.uav_hitbox, &tempState, false, 0.0, false))
  { 
    ROS_WARN("ROUND_SEARCH_STATE: Rounded state is in static or dynamic collision! - Perturb it!");
  }
  return roundedState;
}


/*
This function computes the trajectory prefix that should be connected to the new plan from a given trajectory.
It will compare TrajectoryStates (instead of SearchState see below) from the trajectory with the startState and return the prefix.
*/
std::vector<TrajectoryState> LatticePlannerNode::calculate_trajectory_prefix2(double duration, TrajectoryState &startState, std::vector<TrajectoryState> &trajectory)
{
  std::vector<TrajectoryState> prefix;
  
  // Find the trajectory index at which the prefix should start
  int prefix_start_index = 0;
  while (prefix_start_index < trajectory.size() && trajectory[prefix_start_index].time < duration) {
    prefix_start_index++;
  }
  
  int end_index = prefix_start_index;
  bool found_index = false;
  while (!found_index && end_index < trajectory.size()) 
  {

    if (trajectory[end_index] == startState)
      found_index = true;
    else
      end_index++;
  }

  if (!found_index) 
  {
    std::cerr << "prefix_start_index2 in current_trajectory: " << prefix_start_index << "\n";
    std::cerr << "trajectory[prefix_start_index]2 in current_trajectory: " << trajectory[prefix_start_index] << "\n";
    std::cerr << "We are looking for state2 from new_trajectory: " << startState << "\n";
    int n = 0;
    bool found_state = false;
    for (TrajectoryState &state : trajectory) 
    {
      if (state == startState) 
      {
        found_state = true;
        break;
      }
      n++;
    }

    if (found_state) 
    {
      bool a = prefix_start_index < trajectory.size();
      bool b = trajectory[prefix_start_index].time < duration;
      std::cerr
          << "The state we are looking for exists in the trajectory at index "
          << n << " which is BEFORE prefix_start_index=" << prefix_start_index
          << ". \n";
      if (a)
        std::cerr << "   >prefix_start_index increment stoped due to "
                     "prefix_start_index < trajectory.size()\n";
      if (b)
        std::cerr << "   >prefix_start_index increment stoped due to "
                     "trajectory[prefix_start_index].time < duration\n";
    } else {
      std::cerr << "The state we are looking for does NOT exist in the "
                   "trajectory at all! \n";
    }
    std::cerr << "current_trajectory:\n";
    int k = 0;
    for (TrajectoryState &state : trajectory) {
      std::cerr << k << "] " << state << "\n";
      k++;
    }

    ROS_ERROR_STREAM("calculate_trajectory_prefix: found no end index!");
    abort(); // DEBUG
    return prefix;
  }


  if (end_index == prefix_start_index)
      return prefix;

    if (end_index < prefix_start_index) {
      ROS_ERROR_STREAM("end_index < prefix_start_index");
      return prefix;
    }

    prefix.reserve(end_index - prefix_start_index + 1);
    for (int n = prefix_start_index; n < end_index; n++) 
    {
      prefix.push_back(trajectory[n]);
    }
   
    return prefix;

} 


/*
Given a duration, a startState and a current trajectory, will this function compute the first part of the trajectory
that should be connected to the new plan. This is done by finding the index in the trajectory where the startState
is located. The function will return a vector of TrajectoryStates that should be connected to the new plan.
*/
std::vector<TrajectoryState> LatticePlannerNode::calculate_trajectory_prefix(double duration, State &startState, std::vector<TrajectoryState> &trajectory) 
{
  std::vector<TrajectoryState> prefix;
  SearchState first_state_on_plan = startState.state;

  // Find the trajectory index at which the prefix should start
  int prefix_start_index = 0;
  while (prefix_start_index < trajectory.size() && trajectory[prefix_start_index].time < duration) {
    prefix_start_index++;
  }

  // Find the trajectory index at which the prefix should stop
  int end_index = prefix_start_index;
  bool found_index = false;
  while (!found_index && end_index < trajectory.size()) 
  {
    SearchState tempState = SearchState(trajectory[end_index].position.x(), 
                                        trajectory[end_index].position.y(),
                                        trajectory[end_index].position.z(), 
                                        trajectory[end_index].velocity.x(),
                                        trajectory[end_index].velocity.y(), 
                                        trajectory[end_index].velocity.z());
  
    if (tempState == first_state_on_plan)
      found_index = true;
    else
      end_index++;
  }

  if (!found_index) 
  {
    std::cerr << "prefix_start_index: " << prefix_start_index << "\n";
    std::cerr << "trajectory[prefix_start_index]: " << trajectory[prefix_start_index] << "\n";
    std::cerr << "We are looking for state: " << first_state_on_plan << "\n";
    int n = 0;
    bool found_state = false;
    for (TrajectoryState &state : trajectory) 
    {
      SearchState tempState2 = SearchState(state.position.x(), 
                                           state.position.y(), 
                                           state.position.z(),
                                           state.velocity.x(), 
                                           state.velocity.y(), 
                                           state.velocity.z());

      if (tempState2 == first_state_on_plan) 
      {
        found_state = true;
        break;
      }
      n++;
    }

    if (found_state) 
    {
      bool a = prefix_start_index < trajectory.size();
      bool b = trajectory[prefix_start_index].time < duration;
      std::cerr
          << "The state we are looking for exists in the trajectory at index "
          << n << " which is BEFORE prefix_start_index=" << prefix_start_index
          << ". \n";
      if (a)
        std::cerr << "   >prefix_start_index increment stoped due to "
                     "prefix_start_index < trajectory.size()\n";
      if (b)
        std::cerr << "   >prefix_start_index increment stoped due to "
                     "trajectory[prefix_start_index].time < duration\n";
    } else {
      std::cerr << "The state we are looking for does NOT exist in the "
                   "trajectory at all! \n";
    }
    std::cerr << "Trajectory:\n";
    int k = 0;
    for (TrajectoryState &state : trajectory) {
      std::cerr << k << "] " << state << "\n";
      k++;
    }

    ROS_ERROR_STREAM("calculate_trajectory_prefix: found no end index!");
    abort(); // DEBUG
    return prefix;
  }

  if (end_index == prefix_start_index)
    return prefix;

  if (end_index < prefix_start_index) {
    ROS_ERROR_STREAM("end_index < prefix_start_index");
    return prefix;
  }

  prefix.reserve(end_index - prefix_start_index + 1);
  for (int n = prefix_start_index; n < end_index; n++) 
  {
    prefix.push_back(trajectory[n]);
  }

  return prefix;
}


/*
Return closest state given a trajecotry and spatially-wise.
*/
int LatticePlannerNode::find_closest_trajectory_state(SearchState &state, 
                                                      const std::vector<TrajectoryState> &trajectory,
                                                      int start_index, 
                                                      int stop_index) 
{

  double min_distance = 10e10;
  int min_index = 0;
  double distance;
  for (int n = start_index; n <= stop_index; n++) {
    distance = (state.position - trajectory[n].position).norm();
    ;
    if (distance < min_distance) {
      min_distance = distance;
      min_index = n;
    }
  }
  return min_index;
}


int LatticePlannerNode::next_trajectory_state(SearchState &current_state, 
                                              const std::vector<TrajectoryState> &trajectory,
                                              int start_index, 
                                              int stop_index) 
{

  /* Find the two state-indexes which surrounds the current pose
   * ------------------------------------------------------------*/
  int min_index = find_closest_trajectory_state(current_state, trajectory,
                                                start_index, stop_index);
  int min_index1;
  int min_index2;
  if (min_index == 0) {
    min_index1 = min_index;
    min_index2 = min_index + 1;
  } else if (min_index == trajectory.size() - 1) {
    min_index1 = min_index - 1;
    min_index2 = min_index;
  } else {
    SearchState trajectorySearchState1(
        SearchState(trajectory[min_index - 1].position.x(),
                    trajectory[min_index - 1].position.y(),
                    trajectory[min_index - 1].position.z()));
    SearchState trajectorySearchState2(SearchState(
        trajectory[min_index].position.x(), trajectory[min_index].position.y(),
        trajectory[min_index].position.z()));
    double p_distance = projected_distance(
        trajectorySearchState1, trajectorySearchState2, current_state);
    if (p_distance > 0) {
      min_index1 = min_index - 1;
      min_index2 = min_index;
    } else {
      min_index1 = min_index;
      min_index2 = min_index + 1;
    }
  }
  return min_index2;
}

void LatticePlannerNode::generate_global_plan() 
{

  //Cleanup
  Dji_reference_plan_.primitive_id.clear();
  Dji_reference_plan_.time_from_start.clear();
  Dji_reference_plan_.durations.clear();

  for(vector<State>::iterator state = current_plan_.begin(); state != current_plan_.end()-1; state++) 
  {
    vector<State>::iterator nx = next(state,1);
    Dji_reference_plan_.time_from_start.push_back(ros::Duration(state->time));
    MotionPrimitive m = lattice_planner_.get_primitive(nx->actionID);
    Dji_reference_plan_.primitive_id.push_back(m.ID);
    Dji_reference_plan_.durations.push_back(m.duration);
    geometry_msgs::Point initial_point;
    initial_point.x = current_plan_[0].state.position.x();
    initial_point.y = current_plan_[0].state.position.y();
    initial_point.z = current_plan_[0].state.position.z();
    Dji_reference_plan_.initial_point.push_back(initial_point);
    }
}

void LatticePlannerNode::generate_global_trajectory() 
{

  this->Dji_reference_trajectory_ = trajectory_msgs::MultiDOFJointTrajectory();

  Dji_reference_trajectory_ = trajectory_msgs::MultiDOFJointTrajectory();
  geometry_msgs::Transform transform;
  geometry_msgs::Twist velocities;
  geometry_msgs::Twist accelerations;

  for (int n = 0; n < this->current_trajectory_.size(); n++) {

    trajectory_msgs::MultiDOFJointTrajectoryPoint ref_point;
    transform.translation.x = this->current_trajectory_[n].position.x();
    transform.translation.y = this->current_trajectory_[n].position.y();
    transform.translation.z = this->current_trajectory_[n].position.z();

    velocities.linear.x = this->current_trajectory_[n].velocity.x();
    velocities.linear.y = this->current_trajectory_[n].velocity.y();
    velocities.linear.z = this->current_trajectory_[n].velocity.z();

    double r = this->current_trajectory_[n].roll;
    double p = this->current_trajectory_[n].pitch;
    double y = this->current_trajectory_[n].yaw;

    double liner_drag_coefficient = 0.01;

    double useRefCtrl = 1;
    accelerations.linear.x =
        useRefCtrl * (cos(y) * sin(p) * cos(r) + sin(y) * sin(r)) *
            this->current_trajectory_[n].thrust -
        liner_drag_coefficient * this->current_trajectory_[n].velocity.x();
    accelerations.linear.y =
        useRefCtrl * (sin(y) * sin(p) * cos(r) - cos(y) * sin(r)) *
            this->current_trajectory_[n].thrust -
        liner_drag_coefficient * this->current_trajectory_[n].velocity.y();
    accelerations.linear.z =
        useRefCtrl *
        (cos(p) * cos(r) * this->current_trajectory_[n].thrust - 9.806);

    tf::Quaternion q = tf::createQuaternionFromRPY(r, p, y);

    transform.rotation.x = q[0];
    transform.rotation.y = q[1];
    transform.rotation.z = q[2];
    transform.rotation.w = q[3];

    ref_point.transforms.push_back(transform);
    ref_point.velocities.push_back(velocities);
    ref_point.accelerations.push_back(accelerations);

    ref_point.time_from_start =
        ros::Duration(this->current_trajectory_[n].time);

    Dji_reference_trajectory_.points.push_back(ref_point);
  }
}

void LatticePlannerNode::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) 
{
  if (octree != NULL)
    delete (octree);
  octree = dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(*msg));
  lattice_planner_.update_octomap(octree);
}


void LatticePlannerNode::poseCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  this->dji_state_.position.x() = msg->point.x;
  this->dji_state_.position.y() = msg->point.y;
  this->dji_state_.position.z() = msg->point.z;
  lattice_planner_.uav_hitbox.set_position(msg->point.x, msg->point.y,
                                           msg->point.z);
  publish_dji_hitbox();
  has_valid_pose_ = true;

  // Update SetupScenario continiously
  reached_goal_pub_.publish(lattice_planner_node->reached_goal_msg);
}

void LatticePlannerNode::velCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) 
{
  this->dji_state_.velocity.x() = msg->vector.x;
  this->dji_state_.velocity.y() = msg->vector.y;
  this->dji_state_.velocity.z() = msg->vector.z;
  has_valid_vel_ = true;
}

void LatticePlannerNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  double pos_x = msg->pose.position.x;
  double pos_y = msg->pose.position.y;
  double pos_z = msg->pose.position.z;

  float minimum_height = 2.0; // Default
  float default_height = 2.0; // Default

  if (pos_z == 0.0) 
  {
    pos_z = default_height;
  } 
  else if (pos_z < minimum_height) 
  {
    pos_z = minimum_height;
  }
  
  setGoalState(SearchState(pos_x, pos_y, pos_z, 0, 0, 0));
}

void LatticePlannerNode::setGoalState(SearchState goal_state) 
{
  ROS_INFO_STREAM("Orginal Goal State (Off Lattice): " << goal_state);
  goal_state = round_search_state(goal_state);
  ROS_WARN_STREAM("Goal State after Rounding (On Lattice): " << goal_state);
  this->goal_state_ = goal_state;
  ROS_INFO_STREAM("Goal Updated to: x=" << goal_state.position.x() << " y= "<< goal_state.position.y() << " z= " <<   goal_state.position.z());
  new_nav_goal_ = true;
  has_valid_goal_ = true;
}

/*
Subscribes to the /dji0/scenario_info topic.
Sets the scenario parameters from SetupScenario in the LatticePlanner.
Essentially the same setup as in LatticePlannerNode::apply_scenario
*/
void LatticePlannerNode::scenarioCallback(const lattice_planner::Scenario::ConstPtr& msg)
{

  lattice_planner_node->lattice_planner_.set_allowed_volume(msg->limits.minX, msg->limits.maxX,
                                                            msg->limits.minY, msg->limits.maxY,
                                                            msg->limits.minZ, msg->limits.maxZ
                                                            );

  lattice_planner_node->object_lattice_planner_.set_allowed_volume(msg->limits.minX, msg->limits.maxX,
                                                            msg->limits.minY, msg->limits.maxY,
                                                            msg->limits.minZ, msg->limits.maxZ
                                                            );
  has_valid_scenario_ = true;
}

/*
Subscribes to the /dji0/obstacles topic.
Updates the LatticePlanner on static and dynamic obstacles info by 
reconstructing the objects and feeding them them to the planner.
*/
void LatticePlannerNode::obstacleCallback(const lattice_planner::Obstacles::ConstPtr& msg)
{
  std::vector<lattice_planner::Obstacle> obstacles =  msg->obstacles;

  // Add obstacles once it the obstacles list, then the planner will update them internally
  // so we don't want to mess with the list
  if(!lattice_planner_node->has_valid_obstacles())
  {

    lattice_planner_node->clear_obstacles();
  
    for(int i = 0; i < obstacles.size(); i++)
    {
      
      lattice_planner::Obstacle obstacle = obstacles[i];
      Obstacle obs;

      //Static - wall
      std::string name = obstacle.name;
      std::string wall ("wall");
      if(name.find(wall) != std::string::npos)
      {
        obs = Obstacle(obstacle.position.x,
                      obstacle.position.y,
                      obstacle.position.z,
                      obstacle.radius,
                      obstacle.name
                      );

        obs.get_hitbox().add_AABB(obstacle.radius3D.x, 
                                  obstacle.radius3D.y,
                                  obstacle.radius3D.z
                                  );
      }

      // constantVelocity - simple
      std::string cv ("constantVelocity");
      if(name.compare(cv) == 0)
      {
        StateSpaceModel * model = new ConstantVelocityModel(SearchState(obstacle.state_space_model_position.x,
                                                                        obstacle.state_space_model_position.y,
                                                                        obstacle.state_space_model_position.z,
                                                                        obstacle.state_space_model_velocity.linear.x,
                                                                        obstacle.state_space_model_velocity.linear.y,
                                                                        obstacle.state_space_model_velocity.linear.z
                                                                        ));
        obs = Obstacle(obstacle.position.x,
                       obstacle.position.y,
                       obstacle.position.z,
                       obstacle.radius,
                       obstacle.name,
                       model,
                       0.1,
                       obstacle.id
                      );
      }

      // constantVelocity2 - simple
      std::string cv2 ("constantVelocity2");
      if(name.compare(cv2) == 0)
      {
        StateSpaceModel * model = new ConstantVelocityModel2(SearchState( obstacle.state_space_model_position.x,
                                                                          obstacle.state_space_model_position.y,
                                                                          obstacle.state_space_model_position.z,
                                                                          obstacle.state_space_model_velocity.linear.x,
                                                                          obstacle.state_space_model_velocity.linear.y,
                                                                          obstacle.state_space_model_velocity.linear.z
                                                                          ));
        obs = Obstacle(obstacle.position.x,
                       obstacle.position.y,
                       obstacle.position.z,
                       obstacle.radius,
                       obstacle.name,
                       model,
                       0.1,
                       obstacle.id
                      );
      }

      //human or uav - advanced
      std::string human ("human");
      std::string uav ("uav");
      if(name.compare(human) == 0 || name.compare(uav) == 0)
      {
        
        AABB limits = create_AABB(  obstacle.limits.minX,
                                    obstacle.limits.maxX,
                                    obstacle.limits.minY,
                                    obstacle.limits.maxY,
                                    obstacle.limits.minZ,
                                    obstacle.limits.maxZ
                                );
        
        obs = create_advanced_obstacle( obstacle.position.x,
                                        obstacle.position.y,
                                        obstacle.position.z,
                                        obstacle.max_speed,
                                        obstacle.type,
                                        limits,
                                        obstacle.predictable,
                                        obstacle.id
                                      );
      
        if(!obstacle.trajectory.empty())
        {
          for (int i = 0; i < obstacle.trajectory.size(); i++)
          {
            lattice_planner::TrajectoryState state = obstacle.trajectory[i];
            TrajectoryState traj_state( state.x, 
                                        state.y, 
                                        state.z, 
                                        state.vx, 
                                        state.vy, 
                                        state.vz, 
                                        state.pitch, 
                                        state.roll, 
                                        state.yaw, 
                                        state.thrust, 
                                        state.time);

            obs.plan.push_back(traj_state);
          }
          obs.plan_index = 0;
          obs.previous_time = -1;
        }
      }

      if(obs.state != 0)
      {
        obs.state->reset_time();
      }

      //Add obstacle (static or dynamic) to the motion planner node
      lattice_planner_node->add_obstacle(obs, false);

      // Add static obstacles to the dynamic obstacles motion planner
      if (name.find(wall) != std::string::npos) 
      {
        lattice_planner_node->object_lattice_planner_.add_obstacle(obs);
      }
    }
  }
  else
  {
    // If we have already added the obstacles, update their position and plan from the SetupScenario node
    std::vector<Obstacle> &dynamic_obstacles = lattice_planner_.get_dynamic_obstacles();
    for(int i = 0; i < dynamic_obstacles.size(); i++)
    {
      for(int j = 0; j < obstacles.size(); j++)
      {

        if(dynamic_obstacles[i].id == obstacles[j].id)
        {
          
          // Update new trajectory to follow
          if(!dynamic_obstacles[i].plan.empty())
          {
            dynamic_obstacles[i].plan.clear();
            for (int k = 0; k < obstacles[j].trajectory.size(); k++)
            {
              lattice_planner::TrajectoryState state = obstacles[j].trajectory[k];
              TrajectoryState traj_state( state.x, 
                                          state.y, 
                                          state.z, 
                                          state.vx, 
                                          state.vy, 
                                          state.vz, 
                                          state.pitch, 
                                          state.roll, 
                                          state.yaw, 
                                          state.thrust, 
                                          state.time);
              dynamic_obstacles[i].plan.push_back(traj_state);
            }
          }

          // Move dynamic obstacle according to new position and new plan and update their velocity
          dynamic_obstacles[i].move(obstacles[j].state_space_model_position.x, obstacles[j].state_space_model_position.y, obstacles[j].state_space_model_position.z,
                                    obstacles[j].state_space_model_velocity.linear.x, obstacles[j].state_space_model_velocity.linear.y, obstacles[j].state_space_model_velocity.linear.z);
        }
      }

      //Reset time to make predictions correct
      dynamic_obstacles[i].state->reset_time();
    }
  }

lattice_planner_node->set_valid_obstacles(true);
}


SearchState LatticePlannerNode::draw_random_free_position(AABB limit, bool static_only,
                                              std::default_random_engine &rng) {
  bool free = false;
  SearchState ss;
  while (!free) 
  { 
    double x = std::uniform_real_distribution<double>(limit.getMinX(),
                                                      limit.getMaxX())(rng);
    double y = std::uniform_real_distribution<double>(limit.getMinY(),
                                                      limit.getMaxY())(rng);
    double z = std::uniform_real_distribution<double>(limit.getMinZ(),
                                                      limit.getMaxZ())(rng);
    ss = SearchState(x, y, z);
    ss = lattice_planner_node->round_search_state(ss);
    State s(SearchState(ss.position), 0);
    float proximity_cost = lattice_planner_.environment.evaluate_proximity_cost(lattice_planner_.uav_hitbox, &s, 0.0, static_only);
    free = !isinf(proximity_cost);
  }
  return ss;
}


/*
This function updates the position of the dynamic obstacles in the simulation.

If the dynamic obstacle is advanced, it will use the LatticePlanner to plan a path and 
move the obstacle accordingly.

If the obstacle is not advanced (meaning CV or CV2), the postion is updated in the
obstacle->state->simulate() call the utilizes the correct motion model to move the dynamic obstacle.

This is currently handled in SetupScenario, and then we subscribe to it to get the relevant data.

*/
void LatticePlannerNode::update_obstacles(double time) 
{
  
  std::vector<Obstacle> &obstacles = lattice_planner_.get_dynamic_obstacles();

  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++) 
  {

    if (obstacle->state != 0 && obstacle->is_advanced) 
    {
      
      SearchState current_state = obstacle->state->get_state();

      if (obstacle->plan.empty()) 
      {
        SearchState goalState = draw_random_position(obstacle->limit);
        goalState = round_search_state(goalState);
        SearchState closestSearchState = round_search_state(current_state);
        object_lattice_planner_.set_allowed_volume(obstacle->limit);
        TrajectoryState dummy_state;

        bool plan_found = object_lattice_planner_.do_planning_cycle(closestSearchState, goalState, 0.2, obstacle->plan, 0.0, dummy_state, 0.0);
        obstacle->plan_index = 0;
        obstacle->previous_time = -1;
      } 
      else 
      {
        double dt = 0;
        if (obstacle->previous_time > 0)
        {
          dt = ros::Time::now().toSec() - obstacle->previous_time;
        }
        
        obstacle->previous_time = ros::Time::now().toSec();
        double dist_remaining = dt * obstacle->max_speed;
        
        while (dist_remaining > 0.0 && dt > 0 && !obstacle->plan.empty()) 
        {
          SearchState &state = obstacle->state->get_state();
          SearchState goal(obstacle->plan.front().position);
          SearchState diff = SearchState(goal.position - state.position);
          double distance = diff.position.norm();
          SearchState vector = diff;
          vector.position.normalize();

          if (distance <= dist_remaining) 
          {
            state.position.x() = goal.position.x();
            state.position.y() = goal.position.y();
            state.position.z() = goal.position.z();
            state.velocity.x() = vector.position.x() * obstacle->max_speed;
            state.velocity.y() = vector.position.y() * obstacle->max_speed;
            state.velocity.z() = vector.position.z() * obstacle->max_speed;
            dist_remaining -= distance;
            obstacle->plan.erase(obstacle->plan.begin());
          } 
          else 
          {
            state.position.x() += vector.position.x() * dist_remaining;
            state.position.y() += vector.position.y() * dist_remaining;
            state.position.z() += vector.position.z() * dist_remaining;
            state.velocity.x() = vector.position.x() * obstacle->max_speed;
            state.velocity.y() = vector.position.y() * obstacle->max_speed;
            state.velocity.z() = vector.position.z() * obstacle->max_speed;
            dist_remaining = 0;
          }
        }
      }
      obstacle->state->reset_time(time); 
    } 
    else if (obstacle->state != 0) 
    {
      obstacle->state->simulate(time);
      obstacle->state->reset_time(time);
    }
  }
}

/*
For each dynamic obstacle, predict future hitbox states by
propagating the state space model future in time.
*/
void LatticePlannerNode::observe_obstacles(double time) {
  std::vector<Obstacle> &obstacles = lattice_planner_.get_dynamic_obstacles();
  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++) 
  {
    if (obstacle->state != 0) 
    {
      Hitbox hitbox = obstacle->state->predict(0);                  // Current position of dynamic obstacle
      hitbox.set_radius(0.0);                                       // Assume perfect observation
      obstacle->state->observe_position(time, hitbox);
    }
  }
}


/*
Publish the trajectory in the form of points where each point is trajectory state.

This function consists of two loops, the first loop publish markers from the primary (BLUE)
and secondary (GREEN) search. This is to show which part of the plan belongs to which planning process.

The second part colors the primitives differently. This is to distinguis between the different primitives and their point in time.
*/
void LatticePlannerNode::publish_visualization_plan_trajectory(trajectory_msgs::MultiDOFJointTrajectory &trajectory, int secondary_traj_start_index) 
{
  
  // Setup for marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.pose.orientation.w = 1.0;

  for (int n = 0; n < trajectory.points.size(); n++) 
  {
    trajectory_msgs::MultiDOFJointTrajectoryPoint &tpoint = trajectory.points[n];
    geometry_msgs::Vector3 &translation = tpoint.transforms[0].translation;

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = translation.x;
    point.y = translation.y;
    point.z = translation.z;

    if (secondary_traj_start_index == -1 || n <= secondary_traj_start_index) 
    {
      //COLOR PRIMARY SEARCH BLUE
      color.a = 1.0;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
    } 
    else 
    {
      //COLOR SECONDARY SEARCH GREEN
      color.a = 1.0;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
    }

    marker.points.push_back(point);
    marker.colors.push_back(color);
  }
  
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  plan_trajectory_pub_.publish(marker);

  // --------------------------------------
  
  // Different colors for each primitive
  std::vector<std_msgs::ColorRGBA> colormap;
  
  //RED
  colormap.push_back(std_msgs::ColorRGBA());
  colormap.back().a = 1.0;
  colormap.back().r = 1.0;
  colormap.back().g = 0.0;
  colormap.back().b = 0.0;

  //GREEN
  colormap.push_back(std_msgs::ColorRGBA());
  colormap.back().a = 1.0;
  colormap.back().r = 0.0;
  colormap.back().g = 1.0;
  colormap.back().b = 0.0;

  //BLUE
  colormap.push_back(std_msgs::ColorRGBA());
  colormap.back().a = 1.0;
  colormap.back().r = 0.0;
  colormap.back().g = 0.0;
  colormap.back().b = 1.0;
  
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "primitives";
  marker.id = 0;
  int primitive_ind = 0;
  
  std_msgs::ColorRGBA black;
  black.a = 1.0;
  black.r = 0.0;
  black.g = 0.0;
  black.b = 0.0;

  for (int n = 0; n < trajectory.points.size(); n++) 
  {

    if (primitive_ind < this->current_plan_.size() - 1 && this->current_trajectory_[n].time > this->current_plan_[primitive_ind+1].time) 
    { 
      // If the primitive index is lower than the current plan size or if the time of the current trajectory is greater than that of the plan
      // We move the index forward to be up to date.
      primitive_ind += 1;
    }

    // If the current point in the trajectory is before the first point in the plan, its in the past
    if (this->current_trajectory_[n].time < this->current_plan_[0].time) 
    {
      // Color it black
      marker.colors[n] = black;
    }
    else 
    {
      // Color everyother red, green and blue and then start over. This is to make it easier to separate them.
      marker.colors[n] = colormap[primitive_ind % colormap.size()];
    }
  }

  plan_trajectory_pub_.publish(marker);

}

void LatticePlannerNode::publish_visualization_plan_trajectory_safety_radius(trajectory_msgs::MultiDOFJointTrajectory &trajectory, int secondary_traj_start_index)
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.orientation.w = 1.0;

  for (int n = 0; n < trajectory.points.size(); n++) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint &tpoint =
        trajectory.points[n];
    geometry_msgs::Vector3 &translation = tpoint.transforms[0].translation;

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = translation.x;
    point.y = translation.y;
    point.z = translation.z;

    if (secondary_traj_start_index == -1 || n <= secondary_traj_start_index) 
    {
      // Primary Search is BLUE
      color.a = 0.012;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.5;
    } 
    else 
    {
      // Secondary Search is GREEN
      color.a = 0.012;
      color.r = 0.0;
      color.g = 0.5;
      color.b = 0.0;
    }

    if(this->previous_iteration_was_emergency)
    {
      // Emergency is RED
      color.a = 0.012;
      color.r = 0.5;
      color.g = 0.0;
      color.b = 0.0;
    }

    marker.points.push_back(point);
    marker.colors.push_back(color);
  }
  
  marker.pose.orientation.w = 1.0;
  marker.scale.x = lattice_planner_.uav_safety_radius * 2;
  marker.scale.y = lattice_planner_.uav_safety_radius * 2;
  marker.scale.z = lattice_planner_.uav_safety_radius * 2;
  plan_trajectory_safety_radius_pub_.publish(marker);
}



/*
Visualizing the closed set, i.e the visisted stated in the search process.

The overall effect creates a color transition that:

Starts as red (1,0,0) when time is near 0
Transitions through yellow (1,1,0) as both red and green are active
Ends as green (0,1,0) as time approaches max_time
Finally becomes blue (0,0,1) when time exactly equals max_time

*/
void LatticePlannerNode::publish_visualization_closed_set() 
{
  ClosedSet *closed_set = lattice_planner_.get_closed_set();
  std::vector<State *> states = closed_set->get_states();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner_closed_set";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.orientation.w = 1.0;

  double max_time = 0.0;
  for (int n = 0; n < states.size(); n++) 
  {
    max_time = std::max(max_time, states[n]->time);
  }

  for (int n = 0; n < states.size(); n++) 
  {

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = states[n]->state.position.x();
    point.y = states[n]->state.position.y();
    point.z = states[n]->state.position.z();
    
    color.a = 0.05; //Alpha

    color.r = std::min(1.0, std::max(0.0, 1.0 - states[n]->time / max_time)) * (states[n]->time != max_time);
    color.g = std::min(1.0, std::max(0.0, states[n]->time / max_time)) * (states[n]->time != max_time);
    color.b = 1.0 * (states[n]->time == max_time);

    marker.points.push_back(point);
    marker.colors.push_back(color);

  }

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  closed_set_pub_.publish(marker);

}

void LatticePlannerNode::publish_visualization_octomap(double width,double height,double stepsize) {
  visualization_msgs::MarkerArray markerArray;

  auto qx = this->dji_state_.position.x();
  auto qy = this->dji_state_.position.y();
  auto qz = this->dji_state_.position.z();

  double radius = 1.0;
  double zmin = -1;

  if (octree == NULL) {
    cout << "octree is a null pointer" << endl;
    return;
  }
  int marker_index = 0;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.push_back(marker);

  for (double z = max(zmin, -height); z < height; z += stepsize) {
    if (z == 0) {
      stepsize = octree->getResolution();
    } else {
      stepsize = 1;
    }
    for (double y = -width; y < width; y += stepsize) {
      for (double x = -width; x < width; x += stepsize) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();

        if (z == 0) {
          marker.type = visualization_msgs::Marker::CUBE;
        } else {
          marker.type = visualization_msgs::Marker::SPHERE;
        }
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.orientation.w = 1.0;
        marker.id = marker_index++;

        marker.pose.position.x = roundPartial(x + qx, octree->getResolution());
        marker.pose.position.y = roundPartial(y + qy, octree->getResolution());
        marker.pose.position.z = roundPartial(z + qz, octree->getResolution());


        point3d pmin = point3d(roundPartial(marker.pose.position.x - radius,
                                            octree->getResolution()),
                               roundPartial(marker.pose.position.y - radius,
                                            octree->getResolution()),
                               roundPartial(marker.pose.position.z - radius,
                                            octree->getResolution()));

        point3d pmax = point3d(roundPartial(marker.pose.position.x + radius,
                                            octree->getResolution()),
                               roundPartial(marker.pose.position.y + radius,
                                            octree->getResolution()),
                               roundPartial(marker.pose.position.z + radius,
                                            octree->getResolution()));

        float occupancy = -1;
        for (OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(pmin, pmax),
                                       end = octree->end_leafs_bbx();
             it != end; ++it) {
          if (it->getOccupancy() >= occupancy) {
            occupancy = it->getOccupancy();
          }
          if (occupancy > 0.5) break;
        }

        if (occupancy != -1) {
          if (occupancy > 0.5) {
            marker.ns = "occupied";
          } else {
            marker.ns = "free";
          }
          marker.color.a = 0.5;
          marker.color.r = occupancy;     // std::min(1.0, std::max(0.0, 1.0 -
                                           // states[n]->time/max_time)) *
                                           // (states[n]->time != max_time);
          marker.color.g = 1 - occupancy; // std::min(1.0, std::max(0.0,
                                           // states[n]->time/max_time)) *
                                           // (states[n]->time != max_time);
          marker.color.b = 0; // 1-(occupancy-0.5)*(occupancy-0.5);//1.0 *
                              // (states[n]->time == max_time);
        } else {
          marker.ns = "unknown";
          marker.color.a = 0.5;
          marker.color.r = 0.2;
          marker.color.g = 0.2;
          marker.color.b = 0.2;
        }
        marker.pose.orientation.w = 1.0;
        if (z == 0) {
          marker.ns += "plane";
          marker.scale.x = 0.5;
          marker.scale.y = 0.5;
          marker.scale.z = 0.1;
        } else {
          marker.scale.z = 0.2;
          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
        }
        markerArray.markers.push_back(marker);

        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "safety Radius";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.orientation.w = 1.0;
        marker.id = marker_index++;
        marker.color.a = 0.3;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        markerArray.markers.push_back(marker);
      }
    }
  }
  octomap_pub_.publish(markerArray);
}


void LatticePlannerNode::publish_visualization_octomap_plane_grid() {

  auto z = this->dji_state_.position.z();

  if (octree == NULL) {
    cout << "octree is a null pointer" << endl;
    return;
  }
  float width = 500;
  float radius = 1.0;
  float stepsize = octree->getResolution();
  nav_msgs::OccupancyGrid occupancy_grid;
  occupancy_grid.header.frame_id = "world";
  std::vector<signed char> occupancys;

  for (double y = -width; y < width; y += stepsize) {
    for (double x = -width; x < width; x += stepsize) {
      occupancy_grid.info.resolution = octree->getResolution();

      point3d searchPoint =
          point3d(roundPartial(x, octree->getResolution()),
                  roundPartial(y, octree->getResolution()),
                  roundPartial(z, octree->getResolution()));

      point3d pmin = point3d(
          roundPartial(searchPoint.x() - radius, octree->getResolution()),
          roundPartial(searchPoint.y() - radius, octree->getResolution()),
          roundPartial(searchPoint.z() - radius, octree->getResolution()));

      point3d pmax = point3d(
          roundPartial(searchPoint.x() + radius, octree->getResolution()),
          roundPartial(searchPoint.y() + radius, octree->getResolution()),
          roundPartial(searchPoint.z() + radius, octree->getResolution()));

      // unknown -1
      float occupancy = -1;
      for (OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(pmin, pmax),
                                     end = octree->end_leafs_bbx();
           it != end; ++it) {
        if (it->getOccupancy() >= occupancy) {
          occupancy = it->getOccupancy();
        }
        if (occupancy > 0.5)
          break;
      }
      if(occupancy == -1){
        occupancys.push_back(100);
      }else{
        occupancys.push_back(occupancy*100);
      }
    }
  }
  occupancy_grid.data = occupancys;
  occupancy_grid.info.height=sqrt(occupancys.size());
  occupancy_grid.info.width=sqrt(occupancys.size());
  occupancy_grid.info.origin.position.x = -width - octree->getResolution()/2;
  occupancy_grid.info.origin.position.y = -width - octree->getResolution()/2;
  occupancy_grid.info.origin.position.z = z;

  octomap_plane_occupied_grid_pub_.publish(occupancy_grid);
}

//publish_visualization_vehicle
void LatticePlannerNode::publish_dji_hitbox() 
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 1;
  ;
  marker.action = visualization_msgs::Marker::MODIFY;

  const Hitbox &hitbox = lattice_planner_.uav_hitbox;

  marker.pose.position.x = hitbox.getX();
  marker.pose.position.y = hitbox.getY();
  marker.pose.position.z = hitbox.getZ();

  marker.pose.orientation.w = 1.0;

  marker.color.a = 0.8;
  marker.color.r = 0.3;
  marker.color.g = 0.3;
  marker.color.b = 0.3;

  if (hitbox.has_AABB) {
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = hitbox.aabb.getRX() * 2;
    marker.scale.y = hitbox.aabb.getRY() * 2;
    marker.scale.z = hitbox.aabb.getRZ() * 2;
  } else {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = hitbox.getR() * 2;
    marker.scale.y = hitbox.getR() * 2;
    marker.scale.z = hitbox.getR() * 2;
  }

  dji_hitbox_pub_.publish(marker);
}

//Debugging
void LatticePlannerNode::debug_pose(SearchState state) 
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 1;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.position.x = state.position.x();
  marker.pose.position.y = state.position.y();
  marker.pose.position.z = state.position.z();

  marker.pose.orientation.w = 1.0;

  //Indigo
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.992;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  debug_pose_pub_.publish(marker);
}

void LatticePlannerNode::publish_anchor_point(SearchState state) 
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 1;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.position.x = state.position.x();
  marker.pose.position.y = state.position.y();
  marker.pose.position.z = state.position.z();

  marker.pose.orientation.w = 1.0;

  //Purple
  marker.color.a = 1.0;
  marker.color.r = 0.792;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  anchor_point_pub_.publish(marker);
}

void LatticePlannerNode::publish_off_lattice_pos(SearchState state) 
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 1;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.position.x = state.position.x();
  marker.pose.position.y = state.position.y();
  marker.pose.position.z = state.position.z();

  marker.pose.orientation.w = 1.0;

  //Purple
  marker.color.a = 1.0;
  marker.color.r = 0.965;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  end_point_pub_.publish(marker);
}


void LatticePlannerNode::publish_visualization_obstacles(
    std::vector<Obstacle> obstacles) {
  publish_visualization_obstacles_hitbox(obstacles);
  publish_visualization_obstacles_predictions(obstacles, false);
  publish_visualization_obstacles_predictions(obstacles, true);
}

void LatticePlannerNode::publish_visualization_obstacles_hitbox(std::vector<Obstacle> obstacles) {
  visualization_msgs::MarkerArray markerArray;
  // std::cerr << "!!! publish obstacles !!!\n";
  int marker_index = 0;
  for (int n = 0; n < obstacles.size(); n++) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding box";
    marker.id = marker_index++;
    ;
    marker.action = visualization_msgs::Marker::MODIFY;

    const Obstacle &obstacle = obstacles[n];
    const Hitbox &hitbox = obstacles[n].get_hitbox();

    marker.pose.position.x = hitbox.getX();
    marker.pose.position.y = hitbox.getY();
    marker.pose.position.z = hitbox.getZ();

    marker.pose.orientation.w = 1.0;
    
    if (obstacles[n].display_solid) {
      marker.color.a = 1.0;
    }
    else {
      marker.color.a = 0.5;
    }
    marker.color.r = 1;
    marker.color.g = 0.682;
    marker.color.b = 0.0;

    if (hitbox.has_AABB) {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = hitbox.aabb.getRX() * 2;
      marker.scale.y = hitbox.aabb.getRY() * 2;
      marker.scale.z = hitbox.aabb.getRZ() * 2;
    } else {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = hitbox.getR() * 2;
      marker.scale.y = hitbox.getR() * 2;
      marker.scale.z = hitbox.getR() * 2;
    }

    markerArray.markers.push_back(marker);
  }

  plan_obstacles_pub_.publish(markerArray);
}

/*
Visualizes future states for dynamic obstacles by drawing spheres
for each future state, where the sphere is supposed to represent some
uncertainty connected to it. 

obstacles: list of obstales where future states will be visulized
conservative_predictions: false = small spheres, true = large spehers

*/
void LatticePlannerNode::publish_visualization_obstacles_predictions(std::vector<Obstacle> obstacles, bool conservative_predictions) 
{
  visualization_msgs::MarkerArray markerArray;
  int marker_index = 0;
  for (int n = 0; n < obstacles.size(); n++) 
  {
    if (obstacles[n].state != 0) // If obstacle dynamic
    {
      // Create Predictions
      double dt = 2.0;
      double stopTime = 10.0;
      double old_dt = obstacles[n].dt;
      obstacles[n].dt = dt;
      obstacles[n].clear_predictions();
      int index = 1;
      
      //Make predictions from time to stopTime with timestep dt
      for (double time = 0.0; time <= stopTime; time += dt) 
      {
        const Hitbox &hitbox = obstacles[n].predict(time, conservative_predictions);
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        
        if (conservative_predictions)
          marker.ns = "Predictions (conservative)";
        else
          marker.ns = "Predictions (non-conservative)";
        
        marker.id = marker_index++;
        marker.action = visualization_msgs::Marker::MODIFY;

        marker.pose.position.x = hitbox.getX();
        marker.pose.position.y = hitbox.getY();
        marker.pose.position.z = hitbox.getZ();

        marker.pose.orientation.w = 1.0;

        double alpha = std::max(0.05, 0.4 * (stopTime - time) / stopTime);
        marker.color.a = alpha;
        marker.color.r = 1.0 - alpha;
        marker.color.g = alpha;
        marker.color.b = alpha;

        if (obstacles[n].is_advanced) 
        {
          if (obstacles[n].type == "human") {
            marker.color.b = 0.5;
          }
          if (obstacles[n].type == "uav") {
            marker.color.g = 0.5;
          }
          if (obstacles[n].type == "robot") {
            // marker.color.g = 0.5;
          }
        }

        if (hitbox.has_AABB) 
        {
          marker.type = visualization_msgs::Marker::CUBE;
          marker.scale.x = hitbox.aabb.getRX() * 2;
          marker.scale.y = hitbox.aabb.getRY() * 2;
          marker.scale.z = hitbox.aabb.getRZ() * 2;
        } 
        else 
        {
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.scale.x = hitbox.getR() * 2;
          marker.scale.y = hitbox.getR() * 2;
          marker.scale.z = hitbox.getR() * 2;
        }

        markerArray.markers.push_back(marker);
        index++;
      }

      obstacles[n].dt = old_dt;
      obstacles[n].clear_predictions();
    
    }
  }
  plan_obstacles_pub_.publish(markerArray);
}

void LatticePlannerNode::publish_visualization_obstacle_plan_trajectory(
    std::vector<Obstacle> obstacles) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.orientation.w = 1.0;

  int marker_index = 0;
  for (int n = 0; n < obstacles.size(); n++) {

    if (obstacles[n].state != 0) // Dynamic?
    {
      for (int k = 0; k < obstacles[n].plan.size(); k++) {
        geometry_msgs::Point point;
        std_msgs::ColorRGBA color;

        point.x = obstacles[n].plan[k].position.x();
        point.y = obstacles[n].plan[k].position.y();
        point.z = obstacles[n].plan[k].position.z();

        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;

        marker.points.push_back(point);
        marker.colors.push_back(color);
      }
    }
  }
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  obstacle_plan_trajectory_pub_.publish(marker);
}

void LatticePlannerNode::publish_visualization_travelled_path(){

  geometry_msgs::Point p;
  p.x = this->dji_state_.position.x();
  p.y = this->dji_state_.position.y();
  p.z = this->dji_state_.position.z();

  line_list_.points.push_back(p);
  travelled_path_pub_.publish(line_list_);
}    

/*
Given the current trajectory and duration will this function return if the PRIMARY part of the plan is collision-free or not.
*/
std::tuple<bool, double, TrajectoryState> LatticePlannerNode::plan_in_collision(std::vector<TrajectoryState> current_trajectory, double duration) 
{
  

  /*
  Don't see the point of this code.
  for (int n = 0; n < current_trajectory.size(); n++) {
    current_trajectory[n].time -= duration;
  }
  */
  bool conservative = false;

  return this->lattice_planner_.environment.in_collision(lattice_planner_.uav_hitbox, current_trajectory, 0.0, conservative);
}


/*
Given the current trajectory and duration will this function return if the PRIMARY part of the plan is collision-free or not.
*/
std::tuple<bool, double, TrajectoryState> LatticePlannerNode::plan_in_collision(std::vector<TrajectoryState> current_trajectory, double duration, int secondary_traj_start_index)
{
  
  /*
  Don't see the point of this code.
  for (int n = 0; n < current_trajectory.size(); n++) 
  {
    current_trajectory[n].time -= duration;
  }
  */
  bool conservative = false;

  std::vector<TrajectoryState> dynamic_part_current_traj(current_trajectory.begin(), current_trajectory.begin() + secondary_traj_start_index);
  
  return this->lattice_planner_.environment.in_collision(lattice_planner_.uav_hitbox, dynamic_part_current_traj, 0.0, conservative);
}


/*
Given a plan and a time duration will the function return 
the cost (distance in meters) of the plan, for the states that are within the time duration.
*/
double LatticePlannerNode::calculate_plan_cost(std::vector<State> &previous_plan, double duration) {
  
  // If empty plan, the cost is 0
  if (previous_plan.empty()) 
  {
    return 0.0;
  }

  int n = 0;
  while (n < previous_plan.size() && previous_plan[n].time < duration) 
  {
    n++;
  }

  if (n == 0) 
  {
    return 0.0; // The plan ends before the time ("duration")
  }

  return previous_plan.back().cost_f - previous_plan[n - 1].cost_g;
}


/*
This function returns the index to the first primtive in the current plan that is
of type SECONDARY. If no such primitive exists, it will return the size of the current_plan.
*/
int LatticePlannerNode::calculate_secondary_plan_index(std::vector<State> &current_plan)
{
  int n = 1; 
  while (n < current_plan.size() && 
         lattice_planner_.get_primitive(current_plan[n].actionID).type != MotionPrimitive::SECONDARY) 
  {
    n++;
  }
  return n;
}

int LatticePlannerNode::calculate_secondary_trajectory_start_index(std::vector<TrajectoryState> &current_trajectory, std::vector<State> &current_plan, int secondary_plan_start_index) 
{
  int secondary_start_index = 0;

  int n = current_plan.size() - 1;

  int size_of_current_traj = current_trajectory.size();

  while (n >= secondary_plan_start_index) {
    secondary_start_index += (lattice_planner_.get_primitive(current_plan[n].actionID).trajectory.size() - 1);
    n--;
  }

  return (size_of_current_traj - secondary_start_index);
}


// ---------------------------------- HELPERS DEFINITION -------------------------------------

void shutdown_gracefully() 
{
  std::cout << std::endl << "Shutting down gracefully..." << std::endl;
  lattice_planner_node->logger.write_to_file();

  // Print short summary of run.
  auto &dist_log =
      lattice_planner_node->logger.logs["closest obstacle collision-distance"];
  if (dist_log.size() > 0) {
    double min_dist = 0;
    double num_coll = 0;
    double last_coll_time = 0;
    double dur = dist_log.back().first - dist_log.front().first;

    for (auto &p : dist_log) {
      if (p.second < 0) {
        if (min_dist > p.second)
          min_dist = p.second;
        if (p.first > last_coll_time + 5)
          num_coll++; // Only count collisions in 5s intervals to avoid
                      // double-counting
        last_coll_time = p.first;
      }
    }

    double coll_per_sec = num_coll / dur;
    auto &ttg_log = lattice_planner_node->logger.logs["time to goal"];
    double sum_ttg = 0;
    for (auto &p : ttg_log) {
      sum_ttg += p.second;
    }
    double avg_ttg = sum_ttg / ttg_log.size();
    std::cout << std::endl
              << "Final result: min_dist=" << min_dist
              << " num_coll=" << num_coll << " coll_per_sec=" << coll_per_sec
              << " avg_time_to_goal=" << avg_ttg << " dur=" << dur << std::endl
              << std::endl;
  }
  ros::shutdown();
}

SearchState draw_random_position(AABB limit) 
{
  float x =
      limit.getMinX() +
      static_cast<float>(rand()) /
          (static_cast<float>(RAND_MAX / (limit.getMaxX() - limit.getMinX())));
  float y =
      limit.getMinY() +
      static_cast<float>(rand()) /
          (static_cast<float>(RAND_MAX / (limit.getMaxY() - limit.getMinY())));
  float z =
      limit.getMinZ() +
      static_cast<float>(rand()) /
          (static_cast<float>(RAND_MAX / (limit.getMaxZ() - limit.getMinZ())));
  return SearchState(x, y, z);
}



