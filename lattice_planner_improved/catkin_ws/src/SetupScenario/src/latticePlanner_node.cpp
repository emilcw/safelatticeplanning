#include "latticePlanner_node.h"
#include <map>
#include <algorithm>
#include <random>


LatticePlannerNode *lattice_planner_node; // Unfortunate global variable. Change
                                          // node to nodelet!

double roundPartial(double value, double resolution) {
  return round(value / resolution) * resolution;
}

void shutdown_gracefully() {
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

void mySigintHandler(int sig) { shutdown_gracefully(); }

int main(int argc, char **argv) {

  /*----------------- INIT NODE -------------------*/

  ros::init(argc, argv, "setup_scenario", ros::init_options::NoSigintHandler);
  
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  if (!std::numeric_limits<double>::is_iec559) 
  {
    ROS_ERROR("Inf usage requires IEEE 754 floats!");
    ros::shutdown();
  }

  lattice_planner_node = new LatticePlannerNode();

  // Define node
  ros::NodeHandle nh, private_nh("~");

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

  float location_tolerance = lattice_planner_node->location_tolerance_;
  if (!private_nh.getParam("location_tolerance", location_tolerance)) 
  {
    ROS_ERROR("Using default location_tolerance: ", location_tolerance);
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

  //Setting params for travelled path
  lattice_planner_node->line_list_.header.frame_id = "world";
  lattice_planner_node->line_list_.header.stamp = ros::Time::now();
  lattice_planner_node->line_list_.action = visualization_msgs::Marker::ADD;
  lattice_planner_node->line_list_.pose.orientation.w = 1.0;
  lattice_planner_node->line_list_.id = 0;
  lattice_planner_node->line_list_.type = visualization_msgs::Marker::LINE_STRIP;
  lattice_planner_node->line_list_.scale.x = 0.1;
  lattice_planner_node->line_list_.color.r = 1.0; //RED
  lattice_planner_node->line_list_.color.a = 1.0;

  //Inititally the goal has not been reached
  lattice_planner_node->reached_goal_msg.data = false;

  /*-------------------START PERFORMANCE TESTING-------------------
  mode can be set to:
  single - run a single scenario. Nothing moves
  full   - run all scenarios below. Nothing moves
  live   - run a specific scenario, DJI and dynamic obstacles move.
  eval-prim-exec - evaluate primitive execution
  manual - dji goes into manual mode and waits for nav_goal from user.
  
  random-goals - Sets random goals in the environment (Unclear if it works)
  */

  if ( test == "single" || test == "full")
  {
    ROS_INFO_STREAM("Running performance test in mode: \"" << test << "\"");

    //Writing data from performance test to logt file
    std::string path = log_path + "test_" + log_name + ".txt";
    std::vector<Scenario> scenarios;

    if (test == "single")
    {
      Scenario scenario = create_scenario(scenario_str, test, scenario_parameter, lattice_planner_node->seed, lattice_planner_node->use_predictions);
      scenarios = {scenario};
    }

    if (test == "full")
    {
      scenarios = {
      create_scenario("empty", test, 5, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall", test, 10, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall", test, 50, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall", test, 100, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall3D", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall3D", test, 10, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall3D", test, 50, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("wall3D", test, 100, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("dynamic", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("dynamic2", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("moving_guard", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("corridor", test, -1, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("corridor", test, 2, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
       create_scenario("corridor_fast", test, 0,
                      lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("dead_end", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("indoor_empty", test, -1, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("indoor1", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("indoor_corridor", test, 0,
                      lattice_planner_node->seed,
                      lattice_planner_node->use_predictions),
      create_scenario("blocking_wall", test, 0, lattice_planner_node->seed,
                      lattice_planner_node->use_predictions)};
    }

    //Run the performance test(s)
    std::vector<TestResult> results_new;
    for (int n = 0; n < scenarios.size(); n++) 
    {
      ROS_INFO_STREAM("Running performance test of scenario: \"" << scenarios[n].name << "\"");
      results_new.push_back(lattice_planner_node->run_performance_test(scenarios[n], planning_time, replanning_time));
    }

    //Collect the results from single or full and write to log file
    std::vector<TestResult> results = read_test_results(path);
    results.insert(results.end(), results_new.begin(), results_new.end());
    int new_index = results.size() - results_new.size();
    
    lattice_planner_node->logger.write_to_file();

    std::cout << "==================================\n"
              << "    Performance test complete     \n"
              << "----------------------------------\n";
    std::cout << print_test_results(results, new_index);

    write_test_results(path, results);

    return 0;
  }

  ROS_ERROR_STREAM("Skipping performance testing...");

  Scenario scenario;
  if (test == "eval-prim-exec") 
  {
    ROS_ERROR_STREAM("evaluate-primitive-execution");
    lattice_planner_node->evaluate_primitive_execution();
    return 0;
  }

  ROS_ERROR_STREAM("Skipping eval-prim-exec...");


  if(test == "random-goals")
  {
    //Currently not used since run_test() is not correctly implemented.
    //Should generate random goals the DJI should reach in the scenario.
    ROS_ERROR_STREAM("random-goals");
    run_test(lattice_planner_node, test, scenario_str, planning_time, replanning_time);
    return 0;
  }
  ROS_ERROR_STREAM("Skipping random-goals...");

  if (test == "live")
  {
    ROS_ERROR_STREAM("live");
    ROS_INFO_STREAM("Creating scenario \"" << scenario_str << "\" with parameter = " << scenario_parameter << ".");
    scenario = create_scenario(scenario_str, test, scenario_parameter, lattice_planner_node->seed, lattice_planner_node->use_predictions);
    apply_scenario(lattice_planner_node, scenario);
    //Continue into manual mode to execute scenario
  }


  ROS_INFO_STREAM("Applying scenario..." << scenario.name);
  // ----------------------- GOING INTO LATTICE PLANNER ----------------------------

  ros::Rate loop_rate(10);
  lattice_planner_node->logger.log("loop_rate", 0.0, loop_rate.expectedCycleTime().toSec());

  // Setup dynamic obstacles
  lattice_planner_node->publish_visualization_obstacles();
  loop_rate.sleep();
  lattice_planner_node->initialize_obstacles();

  // ------------- OCTOMAP Setup -------------
  int CountOpendMap = 0;

  // Extract parameters for publish_visualization_octomap
  bool enable_visualization_close_proximity;
  bool enable_visualization_grid;

  if (!private_nh.getParam("enable_visualization_close_proximity", enable_visualization_close_proximity)) {
    ROS_WARN("enable_visualization_close_proximity not set (rosparam)");
  }
  if (!private_nh.getParam("enable_visualization_grid", enable_visualization_grid)) {
    ROS_WARN("enable_visualization_grid not set (rosparam)");
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
  
  // ------------- OCTOMAP DONE -------------

  //Start collisions clock
  std_msgs::Bool planning_started;
  planning_started.data = true;
  lattice_planner_node->clock_start_pub_.publish(planning_started);


  // Simulate Scenario
  while(ros::ok())
  {
    //  Reached goal?
    auto current_state = lattice_planner_node->dji_state_;
    auto goal_state = lattice_planner_node->goal_state_;
    double goal_dist = (current_state.position - goal_state.position).norm();
    
    if (lattice_planner_node->reached_goal_from_planner_)
    {
      ROS_WARN("Planner signaled that it reached goal, terminating experiment...");
      ros::Time before = ros::Time::now();
      while(ros::Time::now() < before + ros::Duration(10))
      {
        //Signal to simulation loop that we have reached the desired pose for 10 seconds
        ROS_INFO("Sending reached goal for 10 more seconds");
        lattice_planner_node->reached_goal_msg.data = true;
        lattice_planner_node->reached_goal_pub_.publish(lattice_planner_node->reached_goal_msg);
      }
      break;
    }
    
    // Visualize static and dynamic objects
    lattice_planner_node->publish_visualization_obstacles();

    // Update dynamic objects
    lattice_planner_node->update_obstacles(ros::Time::now().toSec());

    // Observe objects (Fake)
    lattice_planner_node->observe_obstacles(ros::Time::now().toSec());

    // publish scenario info
    lattice_planner_node->publish_scenario(scenario);

    // Publish obstacles info 
    lattice_planner_node->publish_obstacles_info();

    // Visualize scenario limits
    lattice_planner_node->publish_limits(scenario);
  
    // Octomap Visualization
    if (CountOpendMap++ % 7 == 0){
      if (enable_visualization_close_proximity) lattice_planner_node->publish_visualization_octomap(xy_width,z_width,stepsize);
      if (enable_visualization_grid) lattice_planner_node->publish_visualization_octomap_plane_grid();
    }

    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    ROS_INFO_THROTTLE(5, "Setup complete, Updating dynamic obstacles..."); //Print every 5 seconds
  
    // Collisions
    std_msgs::Bool write_collisions;
    write_collisions.data = true;
    lattice_planner_node->collision_pub_.publish(write_collisions);
  }

  ros::shutdown();
  return 0;
}

// -------------------------------------------------------------------------------

/*
Publishes information about all static and dynamic obstacles on the
/dji0/obtacles topic. This can be utilized by the motion planner to make predictions.
*/
void LatticePlannerNode::publish_obstacles_info()
{
  std::vector<Obstacle> obstacles = lattice_planner_node->get_all_obstacles();
  setup_scenario::Obstacles obstacles_list;

   for(int i = 0; i < obstacles.size(); i++)
    {
      setup_scenario::Obstacle obstacle;
      obstacle.id = i;
      obstacle.name = obstacles[i].get_name();
      obstacle.position.x = obstacles[i].get_hitbox().getX();
      obstacle.position.y = obstacles[i].get_hitbox().getY();
      obstacle.position.z = obstacles[i].get_hitbox().getZ();
      obstacle.radius = obstacles[i].get_hitbox().getR();
      obstacle.motion_model = obstacles[i].get_name();

      if(obstacles[i].state != 0) //If dynamic
      {
        obstacle.state_space_model_position.x = obstacles[i].state->state.position.x();
        obstacle.state_space_model_position.y = obstacles[i].state->state.position.y();
        obstacle.state_space_model_position.z = obstacles[i].state->state.position.z();
        
        obstacle.state_space_model_velocity.linear.x = obstacles[i].state->state.velocity.x();
        obstacle.state_space_model_velocity.linear.y = obstacles[i].state->state.velocity.y();
        obstacle.state_space_model_velocity.linear.z = obstacles[i].state->state.velocity.z();
      }

      obstacle.dt = obstacles[i].dt;
      obstacle.display_solid = obstacles[i].display_solid;
      obstacle.is_advanced = obstacles[i].is_advanced;
      obstacle.type = obstacles[i].type;
      obstacle.max_speed = obstacles[i].max_speed;
      obstacle.limits.minX = obstacles[i].limit.getMinX();
      obstacle.limits.maxX = obstacles[i].limit.getMaxX();
      obstacle.limits.minY = obstacles[i].limit.getMinY();
      obstacle.limits.maxY = obstacles[i].limit.getMaxY();
      obstacle.limits.minZ = obstacles[i].limit.getMinZ();
      obstacle.limits.maxZ = obstacles[i].limit.getMaxZ();

      obstacle.radius3D.x = obstacles[i].get_hitbox().aabb.getRX();
      obstacle.radius3D.y = obstacles[i].get_hitbox().aabb.getRY();
      obstacle.radius3D.z = obstacles[i].get_hitbox().aabb.getRZ();

      obstacle.predictable = obstacles[i].predictable;

      if(!obstacles[i].plan.empty())
      {
        for(int j = 0; j < obstacles[i].plan.size(); j++)
        {
          TrajectoryState state = obstacles[i].plan[j];
          setup_scenario::TrajectoryState trajectory_state;
          trajectory_state.x = state.position.x();
          trajectory_state.y = state.position.y();
          trajectory_state.z = state.position.z();
          trajectory_state.vx = state.velocity.x();
          trajectory_state.vy = state.velocity.y();
          trajectory_state.vz = state.velocity.z();
          trajectory_state.roll = state.roll;
          trajectory_state.pitch = state.pitch;
          trajectory_state.yaw = state.yaw;
          trajectory_state.thrust = state.thrust;
          trajectory_state.time = state.time;
          obstacle.trajectory.push_back(trajectory_state);
        }
      }

      obstacles_list.obstacles.push_back(obstacle);
    }
    
    lattice_planner_node->obstacles_pub_.publish(obstacles_list); 
}

/*
This function publishes information from a given scenario 
on the /dji0/scenario_info topic.
*/
void LatticePlannerNode::publish_scenario(Scenario & scenario)
{
  setup_scenario::Scenario scenario_msg;
  scenario_msg.name = scenario.name;
  scenario_msg.seed = scenario.seed;
  scenario_msg.limits.minX = scenario.limits.getMinX();
  scenario_msg.limits.maxX = scenario.limits.getMaxX();
  scenario_msg.limits.minY = scenario.limits.getMinY();
  scenario_msg.limits.maxY = scenario.limits.getMaxY();
  scenario_msg.limits.minZ = scenario.limits.getMinZ();
  scenario_msg.limits.maxZ = scenario.limits.getMaxZ();
  scenario_msg.start = scenario.start;
  scenario_msg.end = scenario.goal;
  scenario_pub_.publish(scenario_msg);
}

/*
Visualize the scenario limits in RViz.
*/
void LatticePlannerNode::publish_limits(Scenario & scenario)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  double rX = (scenario.limits.getMaxX() - scenario.limits.getMinX())/2;
  double rY = (scenario.limits.getMaxY() - scenario.limits.getMinY())/2;
  double rZ = (scenario.limits.getMaxZ() - scenario.limits.getMinZ())/2;
  double cX =  scenario.limits.getMinX() + rX;
  double cY =  scenario.limits.getMinY() + rY;
  double cZ =  scenario.limits.getMinZ() + rZ;

  marker.pose.position.x = cX;
  marker.pose.position.y = cY;
  marker.pose.position.z = cZ;

  if(rZ != 0)
  {
    marker.scale.x = 2 * rX;
    marker.scale.y = 2 * rY; 
    marker.scale.z = 2 * rZ;
  }
  else
  {
    marker.scale.x = 2 * rX;
    marker.scale.y = 2 * rY; 
    marker.scale.z = 0.1;
  }
 

  marker.color.a = 0.6;
  marker.color.r = 0.224;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  limits_pub_.publish( marker );
}

bool LatticePlannerNode::initialize(ros::NodeHandle &nh,
                                    ros::NodeHandle &private_nh) {
  ROS_INFO("Initializing SetupScenarioNode");
  nh_ = nh;
  private_nh_ = private_nh;

  dji_state_ = SearchState();
  new_nav_goal_ = 0;
  previous_cycle_time_ = ros::Time::now();

  reached_goal_from_planner_ = false;

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

  // The planner waits until a vaild poes and vel message have been sent
  this->has_valid_pose_ = false;
  this->has_valid_vel_ = false;
  this->has_valid_goal_ = false;

  this->last_state_receeding_horizon = false;
  this->last_state_emergency_avoidance = false;

  this->grid_size_ =
      0.50; // OBS: Must correspond to the primitives loaded by lattice_planner_

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
  if (!private_nh.getParam("/dji0/setup_scenario/dji_safety_radius_", uav_safety_radius)) {
    ROS_WARN("dji_safety_radius not set (rosparam)");
    abort();
  }

  bool use_stand_still = true;
  bool use_geometric_secondary_search = true;
  bool use_only_geometric_search = false;
  double plan_duration_minimum = 5.0;
  std::string closed_set_type = "SetWaitTime";
  bool best_effort = true;

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
  if (!private_nh.getParam("use_predictions", this->use_predictions)) {
    ROS_WARN_STREAM("Using obstacles predicted motions as default");
  }

  bool success = lattice_planner_.initialize(
      primitive_path, primitive_amount_in_group, uav_safety_radius, &profiler,
      use_stand_still, use_geometric_secondary_search,
      use_only_geometric_search, plan_duration_minimum, closed_set_type,
      best_effort, true);

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

  // Define publishers
  ROS_INFO("Initializing publisher");
  //trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
  //trajectory_id_pub_ = nh.advertise<setup_scenario::plantime>("plantime", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
  teleport_pub_ = nh.advertise<geometry_msgs::PoseStamped>("teleport/pose",1);

  //Visualization
  //plan_trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_plan_trajectory", 1);
  //plan_trajectory_safety_radius_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_plan_trajectory_safety_radius", 1);
  //plan_vehicle_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_vehicle", 1);
  plan_obstacles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_obstacles", 1);
  //obstacle_plan_trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_obstacles_plan", 1);
  goal_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_goal", 1);
  //travelled_path_pub_ = nh.advertise<visualization_msgs::Marker>("ground_truth/travelled_path",10);
  //closed_set_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_closed_set", 1);
  //octomap_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_openmap", 1);
  //octomap_plane_occupied_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("visualization_occupied_grid", 1);
  reached_goal_pub_ = nh.advertise<std_msgs::Bool>("/dji0/reached_goal", 1);

  //Info to Motion Planner
  nav_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("nav_goal", 1);
  obstacles_pub_ = nh.advertise<setup_scenario::Obstacles>("obstacles", 1);
  scenario_pub_ = nh.advertise<setup_scenario::Scenario>("scenario_info", 1);
  limits_pub_ = nh.advertise<visualization_msgs::Marker>("limits", 1);
  collision_pub_ = nh.advertise<std_msgs::Bool>("/write_log", 1);
  clock_start_pub_ = nh.advertise<std_msgs::Bool>("/clock_start", 1);

  // Define subscribers
  ROS_INFO("Initializing subscribers");
  pose_sub_ = nh.subscribe<geometry_msgs::PointStamped>("dji_sdk/local_position", 1, &LatticePlannerNode::poseCallback, this);
  vel_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 1, &LatticePlannerNode::velCallback, this);
  oct_sub_ = nh.subscribe<octomap_msgs::Octomap>("octomap_full", 1, &LatticePlannerNode::octomapCallback, this);
  reached_goal_planner_ = nh.subscribe<std_msgs::Bool>("/dji0/reached_goal_planner", 1, &LatticePlannerNode::reachedGoalPlannerCallback, this);

  object_lattice_planner_.initialize(primitive_path,
                                     primitive_amount_in_group,
                                     uav_safety_radius,
                                     &object_profiler,
                                     false,
                                     false,
                                     true,
                                     0.0,
                                     "Set",
                                     false);

  ROS_INFO("Initialization done");

  return true;
}

//#define USE_PROXIMITY  // Comment out to use time when calculating prefix when
// replanning

void LatticePlannerNode::add_obstacle(Obstacle o, bool dynamic = true) {

  lattice_planner_.add_obstacle(o);
  if (!dynamic) {
    this->num_static_obstacles++;
  }
  ROS_INFO_STREAM("Added Obstacle: name=" << o.get_name() << " x= " << o.get_hitbox().getX() << " y= " << o.get_hitbox().getY() << " z= " << o.get_hitbox().getZ() << " radius= " << o.get_hitbox().getR());

}

bool LatticePlannerNode::planning_cycle(double planning_time,
                                        double replanning_time) {
  ROS_INFO("Planning_cycle");

  // Option
  bool replanning_enabled = true;
  bool best_effort_receeding_horizon =
      true; // Follow the best partial plan if no plan to goal found
  bool best_effort_emergency_avoidance =
      false; // Live the longest if no collision free plan found
  double cost_to_change_plan = 5.0; // Used by re-planning
  double cost_margin = 2.0;         // Used by best effort behavior
  double distance_margin = 0.5;     // Used by best effort behavior

  SearchState closestSearchState;
  double time_since_last_cycle;
  double planning_duration;
  vector<TrajectoryState> new_trajectory;
  bool plan_found = false;
  bool plan_new = false;
  bool use_old_plan = false;

  // Logging
  double time_of_plan = ros::Time::now().toSec();

  if (new_nav_goal_) {
    ROS_INFO(">>New navigation goal.");
    LOG("New navigation goal", time_of_plan, 1);
  }

  if (is_plan_in_the_past(this->current_plan_)) {
    this->current_plan_.clear();
    this->current_trajectory_.clear();
    ROS_INFO("Plan is in the past.");
    LOG("Plan is in the past", time_of_plan, 1);
  }
  bool has_current_plan = !this->current_trajectory_.empty();

  if (has_current_plan && replanning_enabled) {
    ROS_INFO("Re-planning...");
    LOG("Re-planning", time_of_plan, 1);

    // std::cout << "Current plan (before time adjustmend): \n==============\n";
    // print_plan(this->current_plan_);

    time_since_last_cycle = time_of_plan - this->previous_cycle_time_.toSec();
    time_adjust_plan(this->current_plan_, this->current_trajectory_,
                     time_since_last_cycle);

    std::cerr << "Finding secondary plan start index" << std::endl;

    int secondary_plan_start_index =
        calculate_secondary_plan_index(this->current_plan_);

    bool current_plan_collision_free;
    int primary_plan_end_index = current_plan_.size() - 1;
    if (secondary_plan_start_index != this->current_plan_.size()) {
      // secondary search was used
      std::cerr << "secondary plan start index: " << secondary_plan_start_index
                << std::endl;
      std::cerr << "Size of total plan: " << this->current_plan_.size()
                << std::endl;

      int secondary_trajectory_index =
          calculate_secondary_trajectory_start_index(
              this->current_trajectory_, this->current_plan_,
              secondary_plan_start_index);

      std::cerr << "Secondary traj start index: " << secondary_trajectory_index
                << std::endl;
      std::cerr << "Size of current trajectory: "
                << this->current_trajectory_.size() << std::endl;

      primary_plan_end_index =
          std::min((int)this->current_plan_.size() - 1,
                   std::max(0, secondary_plan_start_index - 1));

      std::cerr << "Length of primary plan end index: "
                << primary_plan_end_index + 1 << std::endl;

      current_plan_collision_free = !plan_in_collision(
          this->current_trajectory_, 0.0, secondary_trajectory_index);
    } else {
      // only primary search was used
      std::cerr << "only primary search was used" << std::endl;
      current_plan_collision_free =
          !plan_in_collision(this->current_trajectory_, 0.0);
    }

    if (!current_plan_collision_free) {
      std::cerr << "\n\n\n\n\n\n\nCurrent plan in collision!\n\n\n\n\n\n\n";
      // abort();
    }

    // bool current_plan_duration_valid = this->current_plan_.back().time >=
    // lattice_planner_.get_plan_duration_minimum();
    bool current_plan_duration_valid =
        this->current_plan_[primary_plan_end_index].time >=
        lattice_planner_.get_plan_duration_minimum();
    if (lattice_planner_.use_only_geometric_search)
      current_plan_duration_valid = true;
    bool current_plan_reach_goal = is_plan_leading_to_goal(
        this->current_plan_, lattice_planner_.goalState);
    bool current_plan_dynamic_only =
        secondary_plan_start_index == this->current_plan_.size();
    // TODO: current_plan_has_positive_timefg = ...
    bool current_plan_valid = current_plan_collision_free &&
                              current_plan_duration_valid &&
                              current_plan_reach_goal; // &&
    // current_plan_dynamic_only;
    if (current_plan_valid) {
      ROS_INFO("Current (running) plan is valid (collision free, fulfills plan "
               "duration requirement and lead to the goal).");
    } else {
      if (!current_plan_collision_free) {
        ROS_INFO(
            "Current (running) plan is no longer valid due to collisions!");
        LOG("Plan no longer valid: collisions", time_of_plan, 1);
      }
      if (!current_plan_duration_valid) {
        ROS_INFO("Current (running) plan is no longer valid due to having too "
                 "short duration!");
        LOG("Plan no longer valid: too short duration", time_of_plan, 1);
      }
      if (!current_plan_reach_goal) {
        ROS_INFO("Current (running) plan is not valid due to it not reaching "
                 "the goal!");
        LOG("Plan not valid: Not reaching the goal", time_of_plan, 1);
      }
      /*
      if(!current_plan_dynamic_only) {
        ROS_INFO("Current (running) plan is not valid due to it not being fully
      dynamic all the way to the goal!");
        LOG("Plan not valid: Not dynamic all the way to the goal", time_of_plan,
      1);
      }
      */
    }

    // std::cout << "Current plan (after time adjustment): \n==============\n";
    // print_plan(this->current_plan_);

    // Find suitable grid point to start searching from (the next on the
    // previous path)
    int index;
    float prefix_duration = 3 * replanning_time + 0.1;
    if (current_plan_duration_valid)
      index = next_path_state(
          prefix_duration,
          this->current_plan_); // OL: THIS MAKES LIFE BETTER FOR US!!!!!!
    else if (!current_plan_collision_free)
      index = next_path_state(prefix_duration,
                              this->current_plan_); // TODO: Adjust 0 such that
                                                    // it reflects how long time
                                                    // we are collision free
    else if (!current_plan_duration_valid)
      index = next_path_state(
          std::min(prefix_duration,
                   (float)std::max(
                       this->current_plan_[primary_plan_end_index].time, 0.0)),
          this->current_plan_);
    // Might not work yet. Timing problems.

    // Replanning debugging
    /* If the previous plan is valid but the UAV fail to track it properly and
       therefore enter a collision (start collision), then there will be no
       previous plan next time and the planning process will not do a
       re-planning.
       OR there is some problems with the time adjustments and where the
       re-planning continues..*/
    // int index = next_path_state(this->dji_state_);
    std::cout << "\n==============\n>>Current Pose: " << this->dji_state_
              << "\n";
    if (index < this->current_plan_.size()) {
      std::cout << ">>Next state has index " << index << " : "
                << this->current_plan_[index] << "\n";
    } else {
      ROS_ERROR_STREAM("next_path_state index is out of bound! index="
                       << index << " but this->current_plan_.size()="
                       << this->current_plan_.size());
    }
    std::cout << ">>Time duration: " << time_since_last_cycle << "\n";
    LOG("Time since last cycle", time_of_plan, time_since_last_cycle);
    std::cout << "==============\n";

    // Seach for plan
    double time_before_planning = time_of_plan;
    closestSearchState = this->current_plan_[index].state;
    double start_time = this->current_plan_[index].time;
    plan_found = lattice_planner_.do_planning_cycle(
        closestSearchState, this->goal_state_, replanning_time, new_trajectory,
        start_time);
    std::vector<State> new_plan = lattice_planner_.clone_solution_path();
    planning_duration = ros::Time::now().toSec() - time_before_planning;
    ROS_INFO_STREAM("planning_duration: " << planning_duration);
    LOG("Planning duration", time_of_plan, planning_duration);

    // Time adjust current plan with planning duration
    time_adjust_plan(this->current_plan_, this->current_trajectory_,
                     planning_duration);
    index = next_path_state(0.0, this->current_plan_);
    // Time adjust new plan with planning duration
    time_adjust_plan(new_plan, new_trajectory, planning_duration);

    bool found_new_plan =
        plan_found && this->is_new_plan(new_plan, this->current_plan_, index);
    if (found_new_plan)
      std::cerr << "\n\n\n\n found_new_plan \n\n\n\n";

    // Keep old plan if it is sill valid and not much more expensive than the
    // new plan
    if (plan_found && current_plan_valid && !new_nav_goal_) {
      double cost_old = calculate_plan_cost(this->current_plan_, 0.0);
      double cost_new = new_plan.back().cost_g;
      ROS_INFO_STREAM(">>Cost old plan: " << cost_old
                                          << ", Cost new plan: " << cost_new);
      if (found_new_plan && cost_old <= cost_new + cost_to_change_plan) {
        ROS_INFO("Found plan, but the old one is still valid and not much more "
                 "expensive.");
        use_old_plan = true;
        LOG("Found plan, but the old one is good enough", time_of_plan, 1);
      }
    } else if (plan_found && current_plan_valid && new_nav_goal_) {
      LOG("Cost new plan:", time_of_plan, new_plan.back().cost_g);
    }

    // Keep old plan if it is still valid and no new plan was found or it is the
    // same as the old one
    if (!found_new_plan && current_plan_valid && !new_nav_goal_) {
      use_old_plan = true;
      if (plan_found && !found_new_plan) {
        ROS_INFO("Found plan, but same as old plan!");
        LOG("Found plan, but same as old plan", time_of_plan, 1);
      } else {
        ROS_INFO("No plan found, but the old one is still valid..");
        LOG("No plan found, but old one is still valid", time_of_plan, 1);
        // Can happen if the UAV get to close to on obstacle. WARNING!
      }
    }
    // Use new plan with appended prefix
    if ((found_new_plan && !use_old_plan) ||
        (!current_plan_valid && plan_found)) {
      if (new_plan.front().time < -0.01) { // Added some margin for lag in ROS
                                           // communication and NMPC processing
                                           // time
        // TODO: Does not work as expected. It is common that the front().time
        // is ~ -0.099 ?
        ROS_WARN_STREAM("Appending prefix to new plan is not possible, new "
                        "plan has diverged from current plan.");
        LOG("Appending prefix to new plan is not possible", time_of_plan, 1);
        previous_cycle_time_ = ros::Time::now();
        return true; // false?
      }
      ROS_INFO("Appending prefix of old plan to new plan.");
      LOG("Appending prefix to new plan", time_of_plan, 1);

      // Calculate trajectory prefix
      // Using time duration since last plan was published
      std::vector<TrajectoryState> trajectory_prefix =
          calculate_trajectory_prefix(0.0, new_plan[0],
                                      this->current_trajectory_);

      std::vector<TrajectoryState> total_trajectory;
      total_trajectory.insert(total_trajectory.end(), trajectory_prefix.begin(),
                              trajectory_prefix.end());
      total_trajectory.insert(total_trajectory.end(), new_trajectory.begin(),
                              new_trajectory.end());
      this->current_trajectory_ = total_trajectory;
      this->current_plan_ = new_plan;

      // Debug
      /*
      std::cout << "Previous plan: \n==============\n";
      print_trajectory(this->current_trajectory_);
      std::cout << "New (partial) plan: \n==============\n";
      print_trajectory(new_trajectory);
      std::cout << "Prefix: \n==============\n";
      print_trajectory(trajectory_prefix);
      std::cout << "Total plan: \n==============\n";
      print_trajectory(total_trajectory);
      std::cout << "\n==============\n>>Current Pose: " << this->dji_state_ <<
      "\n==============\n";
      */

      if (!this->current_plan_.empty()) {
        LOG("Plan cost", time_of_plan, this->current_plan_.back().cost_g);
        LOG("Plan duration", time_of_plan, this->current_plan_.back().time);
      }
      LOG("Plan length", time_of_plan, this->current_plan_.size());
      LOG("Trajectory length", time_of_plan, this->current_trajectory_.size());
      // TODO: Add trajectory length in meters...
      LOG("Plan valid node expansions", time_of_plan,
          lattice_planner_.get_closed_set()->size());
      LOG("Plan frontier size", time_of_plan,
          lattice_planner_.get_frontier().size());
    }
    // Otherwise stop, no feasible plan.
  } else { // OLOAN: Only called the first loop since new nav goals just trigger
           // replanning?
    ROS_INFO("Planning ...");
    this->current_plan_.clear();
    this->current_trajectory_.clear();

    closestSearchState = round_search_state(this->dji_state_);
    double time_before_planning = ros::Time::now().toSec();
    plan_found = lattice_planner_.do_planning_cycle(
        closestSearchState, this->goal_state_, planning_time, new_trajectory,
        0.0);
    planning_duration = ros::Time::now().toSec() - time_before_planning;
    ROS_INFO_STREAM("planning_duration: " << planning_duration);
    LOG("Planning duration", time_of_plan, planning_duration);
    this->current_trajectory_ = new_trajectory;
    this->current_plan_ = lattice_planner_.clone_solution_path();

    // Debug
    std::cout << "Current plan: \n=====================\n";
    print_plan(this->current_plan_);
    // std::cout << "Current trajectory: \n=====================\n";
    // print_trajectory(this->current_trajectory_);
    std::cout << "\n==============\n>> Current Pose: " << this->dji_state_
              << "\n==============\n";
    LOG("Planning", time_of_plan, 1);
    if (!this->current_plan_.empty()) {
      LOG("Plan cost", time_of_plan, this->current_plan_.back().cost_g);
      LOG("Plan duration", time_of_plan, this->current_plan_.back().time);
    }
    LOG("Plan length", time_of_plan, this->current_plan_.size());
    LOG("Trajectory length", time_of_plan, this->current_trajectory_.size());
    // TODO: Add trajectory length in meters...
    LOG("Plan valid node expansions", time_of_plan,
        lattice_planner_.get_closed_set()->size());
    LOG("Plan frontier size", time_of_plan,
        lattice_planner_.get_frontier().size());
  }

  bool success = false;
  if (use_old_plan) {
    ROS_INFO("Use old plan!");
    // Do not publish a new plan
    success = true;
    last_state_receeding_horizon = false;
    last_state_emergency_avoidance = false;
    LOG("Use old plan", time_of_plan, 1);
  } else if (plan_found) {
    ROS_INFO("Plan found!");


    publish_visualization_plan_trajectory(Dji_reference_trajectory_,-1);
    generate_global_trajectory(); // Produces Dji_reference_trajectory_
    generate_global_plan(); // Produces Dji_reference_plan_

    ros::Time current_time = ros::Time::now();

    Dji_reference_plan_.header.stamp = current_time;
    trajectory_id_pub_.publish(Dji_reference_plan_);

    Dji_reference_trajectory_.header.stamp = current_time;
    trajectory_pub_.publish(Dji_reference_trajectory_);

    success = true;
    last_state_receeding_horizon = false;
    last_state_emergency_avoidance = false;
    LOG("Use new plan", time_of_plan, 1);

    // Publish plan for other components
    publish_plan(current_plan_);
  } else {
    // No valid plan
    LOG("No valid plan", time_of_plan, 1);
    ROS_WARN("No Valid plan found, stopping.");
    ROS_FATAL("No available collision-free action! BRACE FOR IMPACT!!1");
    LOG("Collision imminent", time_of_plan, 1);
    stop_command();

    // std::cout << "Total plan: \n==============\n";
    // print_trajectory(this->current_trajectory_);

    success = false;
  }
  previous_cycle_time_ = ros::Time::now();

  // Visualization
  publish_visualization_obstacles(lattice_planner_.get_obstacles());
  publish_visualization_obstacle_plan_trajectory(
      lattice_planner_.get_obstacles());

  int secondary_plan_start_index =
      calculate_secondary_plan_index(this->current_plan_);
  if (secondary_plan_start_index != this->current_plan_.size()) {
    // The plan is partly computed from the secondary planning phase
    int secondary_trajectory_index = calculate_secondary_trajectory_start_index(
        this->current_trajectory_, this->current_plan_,
        secondary_plan_start_index);
    publish_visualization_plan_trajectory(Dji_reference_trajectory_,
                                          secondary_trajectory_index);
    publish_visualization_plan_trajectory_safety_radius(
        Dji_reference_trajectory_, secondary_trajectory_index);
  } else {
    // The complete plan in from the primary planning phase
    publish_visualization_plan_trajectory(Dji_reference_trajectory_, -1);
    publish_visualization_plan_trajectory_safety_radius(
        Dji_reference_trajectory_, -1);
  }

  publish_visualization_goal(goal_state_);

  publish_visualization_travelled_path();


  return success;
}

void LatticePlannerNode::stop_command() {
  // Send an empty plan - making the MPC-controller defaulting to following the
  // last command/pose (as commanded above)
  // TODO: Test this code
  this->current_plan_.clear();
  this->current_trajectory_.clear();
  generate_global_trajectory(); // Produces Dji_reference_trajectory_
  // trajectory_pub_.publish(Dji_reference_trajectory_);

  // Give the MCP-controller the command of staying at the current pose
  geometry_msgs::PoseStamped msg;
  ros::spinOnce(); // Spin once to update this->dji_state_
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = this->dji_state_.position.x();
  msg.pose.position.y = this->dji_state_.position.y();
  msg.pose.position.z = this->dji_state_.position.z();
  msg.pose.orientation.w = 1.0;
  pose_pub_.publish(msg);
}

bool LatticePlannerNode::is_new_plan(std::vector<State> new_plan,
                                     std::vector<State> prev_plan,
                                     int start_index) {

  /*std::cout << "New_plan: \n";
  for(int i = 0; i < new_plan.size(); i++){
    std::cout << new_plan[i] << std::endl;
  }

  std::cout << "\nPrev_plan: \n";
  for(int i = 0; i < prev_plan.size(); i++){
    std::cout << prev_plan[i] << std::endl;
  }

  std::cout << "start index: " << start_index << std::endl;
  */

  // If not same size with adj start_index, the plan is new
  if (new_plan.size() + start_index - prev_plan.size() != 0)
    return true;

  // Something is wrong
  if (new_plan.size() == 0 || prev_plan.size() == 0) {
    ROS_WARN("Something is sevearly wrong");
    return true;
  }

  // All ok!
  for (int n = 0; n < new_plan.size(); n++) {
    if (new_plan[n].state != prev_plan[n + start_index].state)
      return true;
  }

  return false;
}

int LatticePlannerNode::next_path_state(double time_duration,
                                        std::vector<State> &plan) {
  if (plan.empty())
    return 0;
  int index = 0;
  while (index < plan.size() - 1 && plan[index].time <= time_duration) {
    index++;
  }
  return index;
}

void LatticePlannerNode::time_adjust_plan(
    std::vector<State> &plan, std::vector<TrajectoryState> &trajectory,
    double dt) {
  for (int n = 0; n < plan.size(); n++)
    plan[n].time -= dt;
  for (int n = 0; n < trajectory.size(); n++)
    trajectory[n].time -= dt;
}

double projected_distance(SearchState &s1, SearchState &s2, SearchState &p) {
  if ((s1.position - s2.position).squaredNorm() < 1e-3)
    return 0.0;
  SearchState state = s1;
  state.position -= s2.position;
  state.position.normalize();
  return state.position.dot(p.position - s1.position);
}

double projected_distance(State *s1, State *s2, SearchState &p) {
  return projected_distance(s1->state, s2->state, p);
}

int find_closest_path_state(SearchState state,
                            const std::vector<State *> &path) {

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

SearchState LatticePlannerNode::round_search_state(SearchState state) {

  SearchState roundedState(state);

  roundedState.position.x() =
      round(state.position.x() / grid_size_) * grid_size_;
  roundedState.position.y() =
      round(state.position.y() / grid_size_) * grid_size_;
  roundedState.position.z() =
      round(state.position.z() / grid_size_) * grid_size_;
  roundedState.velocity.x() = 0.0 * round(state.velocity.x());
  roundedState.velocity.y() = 0.0 * round(state.velocity.y());
  roundedState.velocity.z() = 0.0 * round(state.velocity.z());

  return roundedState;
}

std::vector<TrajectoryState> LatticePlannerNode::calculate_trajectory_prefix(
    double duration, State &startState,
    std::vector<TrajectoryState> &trajectory) {
  std::vector<TrajectoryState> prefix;

  SearchState first_state_on_plan = startState.state;

  // Find the trajectory index at which the prefix should start
  int prefix_start_index = 0;
  while (prefix_start_index < trajectory.size() &&
         trajectory[prefix_start_index].time < duration) {
    prefix_start_index++;
  }

  // Find the trajectory index at which the prefix should stop
  int end_index = prefix_start_index;
  bool found_index = false;
  while (!found_index && end_index < trajectory.size()) {
    SearchState tempState = SearchState(
        trajectory[end_index].position.x(), trajectory[end_index].position.y(),
        trajectory[end_index].position.z(), trajectory[end_index].velocity.x(),
        trajectory[end_index].velocity.y(), trajectory[end_index].velocity.z());
    if (tempState == first_state_on_plan)
      found_index = true;
    else
      end_index++;
  }

  if (!found_index) {
    std::cerr << "end_index: " << end_index << "\n";
    std::cerr << "first_state_on_plan: " << first_state_on_plan << std::endl;
    int n = 0;
    bool found_state = false;
    for (TrajectoryState &state : trajectory) {
      SearchState tempState2 = SearchState(
          state.position.x(), state.position.y(), state.position.z(),
          state.velocity.x(), state.velocity.y(), state.velocity.z());
      if (tempState2 == first_state_on_plan) {
        found_state = true;
        break;
      }
      n++;
    }
    if (found_state) {
      bool a = prefix_start_index < trajectory.size();
      bool b = trajectory[prefix_start_index].time < duration;
      std::cerr
          << "The state we are looking for exists in the trajectory at index "
          << n << " which is BEFORE prefix_star_index=" << prefix_start_index
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
  for (int n = prefix_start_index; n < end_index; n++) {
    prefix.push_back(trajectory[n]);
  }

  /* No longer necessary. trajectory is time adjusted already
  // Remove start time so that start time is 0.0
  double start_time = prefix.front().time - 0.0; // Make the first point to be
  at time 0.0
  for(int n = 0; n < prefix.size(); n++) {
    prefix[n].time -= start_time;
  }
  prefix[0].time = 0.0; // Forcing the first point to be at time 0.0
  */

  return prefix;
}

int LatticePlannerNode::find_closest_trajectory_state(
    SearchState &state, const std::vector<TrajectoryState> &trajectory,
    int start_index, int stop_index) {

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

int LatticePlannerNode::next_trajectory_state(
    SearchState &current_state, const std::vector<TrajectoryState> &trajectory,
    int start_index, int stop_index) {

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

void LatticePlannerNode::generate_global_plan() {

  Dji_reference_plan_.primitive_id.clear();
  Dji_reference_plan_.time_from_start.clear();
  Dji_reference_plan_.durations.clear();
  for(vector<State>::iterator state = current_plan_.begin(); state != current_plan_.end()-1; state++) {
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
  if(current_plan_.size() > 0) {
  }

}
void LatticePlannerNode::generate_global_trajectory() {

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
    velocities.linear.x = this->current_trajectory_[n].velocity.y();
    velocities.linear.x = this->current_trajectory_[n].velocity.z();

    /*
     *
     *  R = [cos(yaw)*cos(pitch)
     * cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw)
     * (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)); ...
     *       cos(pitch)*sin(yaw)
     * cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)
     * (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)); ...
     *      -sin(pitch)                            cos(pitch)*sin(roll)
     * cos(pitch)*cos(roll)];
     *
     */

    double r = this->current_trajectory_[n].roll;
    double p = this->current_trajectory_[n].pitch;
    double y = this->current_trajectory_[n].yaw;

    double liner_drag_coefficient = 0.01;

    // OL: 1 or 0
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

void LatticePlannerNode::octomapCallback(
    const octomap_msgs::Octomap::ConstPtr &msg) {
  // octomap_msgs::readTree(&octree,*msg);
  if (octree != NULL)
    delete (octree);
  octree = dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(*msg));
  lattice_planner_.update_octomap(octree);
}

void LatticePlannerNode::reachedGoalPlannerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  reached_goal_from_planner_ = msg->data;
}

void LatticePlannerNode::poseCallback(
    const geometry_msgs::PointStamped::ConstPtr &msg) {
  // ROS_INFO_STREAM_THROTTLE(1, "positionCallback" << msg->point);
  has_valid_pose_ = true;
  this->dji_state_.position.x() = msg->point.x;
  this->dji_state_.position.y() = msg->point.y;
  this->dji_state_.position.z() = msg->point.z;
  lattice_planner_.uav_hitbox.set_position(msg->point.x, msg->point.y,
                                           msg->point.z);
  publish_visualization_vehicle();
  //ROS_INFO_STREAM_THROTTLE(1, ">> " << this->dji_state_);
  reached_goal_pub_.publish(lattice_planner_node->reached_goal_msg);
}

void LatticePlannerNode::velCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
  // ROS_INFO_STREAM_THROTTLE(1, "positionCallback" << msg->pose);
  has_valid_vel_ = true;
  this->dji_state_.velocity.x() = msg->vector.x;
  this->dji_state_.velocity.y() = msg->vector.y;
  this->dji_state_.velocity.z() = msg->vector.z;
}

void LatticePlannerNode::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  double pos_x = msg->pose.position.x; // Because planner can only take integers.
  double pos_y = msg->pose.position.y;
  double pos_z = msg->pose.position.z;

  // float minimum_height = 12.0;  // Flight test outdoors
  // float default_height = 12.0;  // Flight test outdoors
  float minimum_height = 2.0; // Default
  float default_height = 2.0; // Default
  // float minimum_height = 1.5;
  // float default_height = 1.5;

  if (pos_z == 0.0) { // OLOAN: Set from rviz 2D Nav Goal it will be 0.0, want
                      // reasonable altitude.
    pos_z = default_height;
  } else if (pos_z < minimum_height) { // Min.
    pos_z = minimum_height;
  }
  
  setGoalState(SearchState(pos_x, pos_y, pos_z, 0, 0, 0));
}

void LatticePlannerNode::setGoalState(SearchState goal_state) {
  ROS_INFO_STREAM("Orginal Goal State: " << goal_state);
  goal_state = round_search_state(goal_state);
  ROS_WARN_STREAM("Goal State after Rounding: " << goal_state);
  this->goal_state_ = goal_state;
  ROS_INFO_STREAM("Goal Updated to: x=" << goal_state.position.x() << " y= "<< goal_state.position.y() << " z= " <<   goal_state.position.z());

  new_nav_goal_ = true;
  has_valid_goal_ = true;

  publish_visualization_goal(goal_state);
}

/// REMOVE?
void LatticePlannerNode::obstCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

  geometry_msgs::Pose msg2;
  msg2 = msg->pose.pose;
  update_obstacle(msg2);
}

/// REMOVE?
void LatticePlannerNode::obstCallback2(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  //    ROS_INFO_THROTTLE(1, "Obstacle updated: x=%4.2f, y=%4.2f, z=%4.2f",
  //    obst.x, obst.y, obst.z);
  geometry_msgs::Pose msg2;
  msg2 = msg->pose;
  update_obstacle(msg2);
}

// BYT UT MOT updateHumanPositions (Smartare)
void LatticePlannerNode::update_obstacle(geometry_msgs::Pose p) {
  //    ROS_INFO_THROTTLE(1, "Obstacle updated: x=%4.2f, y=%4.2f, z=%4.2f",
  //    obst.x, obst.y, obst.z);

  std::vector<Obstacle> &obstacles = lattice_planner_.get_dynamic_obstacles();
  double obst_x = p.position.x;
  double obst_y = p.position.y;
  double obst_z = 1.2;
  int dyn_idx = 0;
  int obst_idx = dyn_idx;
  obstacles[obst_idx].move(obst_x, obst_y, obst_z);
  ROS_INFO_THROTTLE(1, "Obstacle %d updated: x=%4.2f, y=%4.2f, z=%4.2f",
                    obst_idx, obst_x, obst_y, obst_z);
  lattice_planner_.print_obstacles();
}

void LatticePlannerNode::initialize_obstacles() {
  std::vector<Obstacle> &obstacles = lattice_planner_.get_dynamic_obstacles();
  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin();
       obstacle != obstacles.end(); obstacle++) {
    if (obstacle->state != 0) {
      obstacle->state->reset_time();
    }
  }
}

SearchState draw_random_position(AABB limit) {
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

SearchState
LatticePlannerNode::draw_random_free_position(AABB limit, bool static_only,
                                              std::default_random_engine &rng) {
  bool free = false;
  SearchState ss;
  while (!free) { // Rejection sampling.
    double x = std::uniform_real_distribution<double>(limit.getMinX(),
                                                      limit.getMaxX())(rng);
    double y = std::uniform_real_distribution<double>(limit.getMinY(),
                                                      limit.getMaxY())(rng);
    double z = std::uniform_real_distribution<double>(limit.getMinZ(),
                                                      limit.getMaxZ())(rng);
    //    ss = draw_random_position(limit);
    ss = SearchState(x, y, z);
    ss = lattice_planner_node->round_search_state(ss);
    State s(SearchState(ss.position), 0);
    float proximity_cost = lattice_planner_.environment.evaluate_proximity_cost(
        lattice_planner_.uav_hitbox, &s, 0.0, static_only);
    free = !isinf(proximity_cost);
    // free =
    // !lattice_planner_.environment.in_collision(lattice_planner_.uav_hitbox,
    // &s, 0.0, static_only); // TODO: What does time here really affect?
    // Answer: Predicted state
  }
  return ss;
}

void LatticePlannerNode::update_obstacles(double time) {
  std::vector<Obstacle> &obstacles = lattice_planner_.get_dynamic_obstacles();

  std::vector<State> temp;

  int nDynamic = 0;
  int nAdvanced = 0;
  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin();
       obstacle != obstacles.end(); obstacle++) {
    if (obstacle->state != 0)
      nDynamic++;
    if (obstacle->is_advanced)
      nAdvanced++;
  }
  // ROS_WARN_STREAM("nDynamic: " << nDynamic << ", nAdvanced: " << nAdvanced);

  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin();
       obstacle != obstacles.end(); obstacle++) {

    if (obstacle->state != 0 && obstacle->is_advanced) {
      // ROS_WARN("Updating advanced obstacle");
      SearchState current_state = obstacle->state->get_state();
      if (obstacle->plan.empty()) {
        // ROS_WARN("Advanced obstacle need new plan.. ");
        temp.clear();
        // SearchState goalState = SearchState(15,0,1.5);
        SearchState goalState = draw_random_position(obstacle->limit);
        goalState = round_search_state(goalState);
        SearchState closestSearchState = round_search_state(current_state);
        // ROS_WARN_STREAM("Advanced obstacle has new goal: " << goalState);
        object_lattice_planner_.set_allowed_volume(obstacle->limit);
        bool plan_found = object_lattice_planner_.do_planning_cycle(
            closestSearchState, goalState, 0.2, obstacle->plan, 0.0);
        // ROS_WARN_STREAM("Advanced obstacle planning is a success?: " <<
        // plan_found);
        obstacle->plan_index = 0;
        obstacle->previous_time = -1;
      } else {
        // ROS_WARN("Advanced obstacle follow its plan.. ");
        double dt = 0;
        if (obstacle->previous_time > 0)
          dt = ros::Time::now().toSec() - obstacle->previous_time;
        obstacle->previous_time = ros::Time::now().toSec();

        double dist_remaining = dt * obstacle->max_speed;
        while (dist_remaining > 0.0 && dt > 0 && !obstacle->plan.empty()) {

          SearchState &state = obstacle->state->get_state();
          // Draw intermediate goal
          SearchState goal(obstacle->plan.front().position);

          // ROS_WARN_STREAM("Advanced obstacle distance to next way point: " <<
          // distance);
          SearchState diff = SearchState(goal.position - state.position);
          double distance = diff.position.norm();
          SearchState vector = diff;
          vector.position.normalize();

          if (distance <= dist_remaining) {
            // Can go directly to intermediate goal.
            state.position.x() = goal.position.x();
            state.position.y() = goal.position.y();
            state.position.z() = goal.position.z();
            state.velocity.x() = vector.position.x() * obstacle->max_speed;
            state.velocity.y() = vector.position.y() * obstacle->max_speed;
            state.velocity.z() = vector.position.z() * obstacle->max_speed;
            dist_remaining -= distance;
            obstacle->plan.erase(obstacle->plan.begin());
          } else {
            state.position.x() += vector.position.x() * dist_remaining;
            state.position.y() += vector.position.y() * dist_remaining;
            state.position.z() += vector.position.z() * dist_remaining;
            state.velocity.x() = vector.position.x() * obstacle->max_speed;
            state.velocity.y() = vector.position.y() * obstacle->max_speed;
            state.velocity.z() = vector.position.z() * obstacle->max_speed;
            dist_remaining = 0;
          }

          // SearchState state = obstacle->state->get_state();
          // ROS_WARN_STREAM("Advanced obstacle is moving.. from: " << state <<
          // " to: " << goal);
          // ROS_WARN_STREAM("Advanced obstacle is moving.. is at : " <<
          // current_state);
          // ROS_WARN_STREAM("Advanced obstacle is at next way point (index " <<
          // obstacle->plan_index << ").. ");
        }
        //        ROS_INFO("OBSTACLE UPDATE dt=%4.2f, max_speed=%4.2f,
        //        dist_remaining=%4.2f\n", dt, obstacle->max_speed,
        //        dist_remaining);
      }
      obstacle->state->reset_time(time); // TODO: This has no function? Time
                                         // duplicated in obstacle and state
                                         // space model?
    } else if (obstacle->state != 0) {
      obstacle->state->simulate(time);
      obstacle->state->reset_time(time);
    }
  }
}

void LatticePlannerNode::observe_obstacles(double time) {
  std::vector<Obstacle> &obstacles = lattice_planner_.get_dynamic_obstacles();
  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin();
       obstacle != obstacles.end(); obstacle++) {
    if (obstacle->state != 0) {
      Hitbox hitbox =
          obstacle->state->predict(0); // Fake observation at predicted position
      hitbox.set_radius(0.0);          // Assume perfect observation
      obstacle->state->observe_position(time, hitbox);
      obstacle->move(hitbox.getX(), hitbox.getY(),
                     hitbox.getZ()); // Update object hitbox position
    }
  }
}

void LatticePlannerNode::add_advanced_obstacle(double x, double y, double z,
                                               double maxSpeed,
                                               std::string type, AABB limit) {
  float radius = 1.0;
  StateSpaceModel *model =
      new ConstantVelocityModel2(SearchState(x, y, z, 0, 0, 0), radius);
  Obstacle advanced(x, y, z, radius, type, model);
  advanced.is_advanced = true;
  advanced.type = type;
  advanced.max_speed = maxSpeed;
  advanced.limit = limit;

  if (!this->use_predictions) {
    advanced.predictable = false;
  }

  add_obstacle(advanced, false);
}

void LatticePlannerNode::publish_visualization_plan_trajectory (
    trajectory_msgs::MultiDOFJointTrajectory &trajectory,
    int secondary_traj_start_index) {
  
  // All with the same color  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = 1;
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

    if (secondary_traj_start_index == -1 || n <= secondary_traj_start_index) {
      color.a = 1.0;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
    } else {
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
  
  // Different colors for each primitive
  std::vector<std_msgs::ColorRGBA> colormap;
  colormap.push_back(std_msgs::ColorRGBA());
  colormap.back().a = 1.0;
  colormap.back().r = 1.0;
  colormap.back().g = 0.0;
  colormap.back().b = 0.0;
  colormap.push_back(std_msgs::ColorRGBA());
  colormap.back().a = 1.0;
  colormap.back().r = 0.0;
  colormap.back().g = 1.0;
  colormap.back().b = 0.0;
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
  std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
  for (int n = 0; n < trajectory.points.size(); n++) {
    if (primitive_ind < this->current_plan_.size()-1 &&
        this->current_trajectory_[n].time > this->current_plan_[primitive_ind+1].time) {
      primitive_ind += 1;
    }
    if (this->current_trajectory_[n].time < this->current_plan_[0].time) {
      marker.colors[n] = color;
    }
    else {
      marker.colors[n] = colormap[primitive_ind % colormap.size()];
    }
  }
  plan_trajectory_pub_.publish(marker);
}

void LatticePlannerNode::publish_visualization_plan_trajectory_safety_radius(
    trajectory_msgs::MultiDOFJointTrajectory &trajectory,
    int secondary_traj_start_index) {
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

    if (secondary_traj_start_index == -1 || n <= secondary_traj_start_index) {
      color.a = 0.012;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.5;
    } else {
      color.a = 0.012;
      color.r = 0.0;
      color.g = 0.5;
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

#include <algorithm>
void LatticePlannerNode::publish_visualization_closed_set() {
  ClosedSet *closed_set = lattice_planner_.get_closed_set();
  std::vector<State *> states = closed_set->get_states();

  // Debug
  // std::sort(states.begin(), states.end(), ClosedSetSet::LessThan());
  // std::sort(states.begin(), states.end(), ClosedSetSetWaitTime::LessThan());

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner_closed_set";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.orientation.w = 1.0;

  double max_time = 0.0;
  for (int n = 0; n < states.size(); n++) {
    max_time = std::max(max_time, states[n]->time);
  }

  for (int n = 0; n < states.size(); n++) {

    // if(states[n]->state.vy <= 0) continue;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = states[n]->state.position.x();
    point.y = states[n]->state.position.y();
    point.z = states[n]->state.position.z();

    color.a = 0.05;
    // color.a = 0.1;
    color.r = std::min(1.0, std::max(0.0, 1.0 - states[n]->time / max_time)) *
              (states[n]->time != max_time);
    color.g = std::min(1.0, std::max(0.0, states[n]->time / max_time)) *
              (states[n]->time != max_time);
    color.b = 1.0 * (states[n]->time == max_time);

    marker.points.push_back(point);
    marker.colors.push_back(color);

    // Debug
    // std::cout << "wt: " << states[n]->wait_time << ", (" << point.x << ", "
    // << point.y << ", " << point.z << "): " << states[n]->state.vx << ", " <<
    // states[n]->state.vy << ", " << states[n]->state.vz << "\n";
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

void LatticePlannerNode::publish_visualization_vehicle() {
  // std::cerr << "!!! publish vehivcle marker !!!\n";

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

  plan_vehicle_pub_.publish(marker);
}

void LatticePlannerNode::publish_visualization_obstacles(std::vector<Obstacle> obstacles) 
{
  publish_visualization_obstacles_hitbox(obstacles);
  publish_visualization_obstacles_predictions(obstacles, false);
  publish_visualization_obstacles_predictions(obstacles, true);
}

void LatticePlannerNode::publish_visualization_obstacles_hitbox(
    std::vector<Obstacle> obstacles) {
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
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;

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

void LatticePlannerNode::publish_visualization_obstacles_predictions(std::vector<Obstacle> obstacles, bool conservative_predictions) {
  visualization_msgs::MarkerArray markerArray;
  // std::cerr << "!!! publish obstacles !!!\n";
  int marker_index = 0;
  for (int n = 0; n < obstacles.size(); n++) {

    if (obstacles[n].state != 0) // Dynamic?
    {
      // Predictions
      double dt = 2.0;
      double stopTime = 10.0;
      double old_dt = obstacles[n].dt;
      obstacles[n].dt = dt;
      obstacles[n].clear_predictions();
      int index = 1;
      for (double time = 0.0; time <= stopTime; time += dt) {
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
        marker.color.r = 0.5 - alpha;
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


void LatticePlannerNode::publish_visualization_goal(SearchState &state) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lattice_planner";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::MODIFY;

  marker.pose.position.x = state.position.x();
  marker.pose.position.y = state.position.y();
  marker.pose.position.z = state.position.z();

  marker.pose.orientation.w = 1.0;

  marker.color.a = 0.2;
  if (lattice_planner_.isGoalInCollision()) {
    marker.color.r = 0.5;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  } else if (lattice_planner_.isStartInCollision()) {
    marker.color.r = 0.25;
    marker.color.g = 0.25;
    marker.color.b = 0.0;
  } else {
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
  }

  marker.scale.x = lattice_planner_.uav_safety_radius * 2;
  marker.scale.y = lattice_planner_.uav_safety_radius * 2;
  marker.scale.z = lattice_planner_.uav_safety_radius * 2;

  goal_pub_.publish(marker);
}

void run_test(LatticePlannerNode *lattice_planner_node, std::string test,
              std::string scenario, double planning_time,
              double replanning_time) {
  // Only one test right now

  // Wait for subscribers to connect (For visualization)
  ros::spinOnce();
  ros::Rate inv_sleep_time(1);
  inv_sleep_time.sleep();
  ros::spinOnce();

  // Plan
  lattice_planner_node->planning_cycle(planning_time, replanning_time);

  // Visualize updated objects
  lattice_planner_node->publish_visualization_obstacles();

  lattice_planner_node->publish_visualization_closed_set();

  if (test == "log") {
    lattice_planner_node->logger.write_to_file();
    return;
  }

  ros::spin(); // test == "forever" and default
  return;

  if (test == "random_goal") {
    ROS_ERROR("Not supported yet");
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      // ROS_INFO_STREAM("closeness to target: " <<
      // (lattice_planner_node->goal_state_ -
      // lattice_planner_node->dji_state_).length());
      // ROS_INFO_STREAM("target: " << lattice_planner_node->goal_state_);
      // ROS_INFO_STREAM("state: " << lattice_planner_node->dji_state_);

      // Update dynamic objects
      lattice_planner_node->update_obstacles(ros::Time::now().toSec());
      // Observe objects (Fake)
      lattice_planner_node->observe_obstacles(ros::Time::now().toSec());

      double time_now = ros::Time::now().toSec();
      lattice_planner_node->publish_visualization_obstacles();
      if (lattice_planner_node->has_valid_state()) {
        lattice_planner_node->planning_cycle(planning_time, replanning_time);
        lattice_planner_node->new_nav_goal_ = false;
        log_obstacle_collisions(time_now, lattice_planner_node);

        /*
        if(lattice_planner_node->is_at_goal()) {
          ROS_INFO("At goal!");
          ros::spin();
        }
        */

        // Draw loop_ratenew goal if at goal
        // if(log_name != "" && ros::Time::now().toSec() > 120)
        //  return 0;
        /*
if(log_name != "" &&
uav_position.distance(lattice_planner_node->lattice_planner_.goalState.state) &&
 !lattice_planner_node->current_plane_.empty() &&
 lattice_planner_node->current_plane_.back()->time <=
0.9+lattice_planner_node->lattice_planner_.plan_duration_minimum) {
AABB limit = create_AABB(-15, 55, -35, 15, 1.5, 3.5);
SearchState goalState = draw_random_position(limit);
// Simulate goal reading (GOAL POSITION)
geometry_msgs::PoseStamped * goal_position = new geometry_msgs::PoseStamped();
goal_position->header.stamp = ros::Time::now();
goal_position->pose.position = create_point(goalState.x, goalState.y,
goalState.z);
geometry_msgs::PoseStamped::ConstPtr goal_ptr(goal_position);
lattice_planner_node->goalCallback(goal_ptr);

}
  */

      } else {
        ROS_WARN_STREAM_THROTTLE(1, "The planner is missing valid pose or "
                                    "valid velocity. Cannot plan.");
      }
      ros::spinOnce();
      loop_rate.sleep();
      ros::spinOnce();
      ROS_INFO_THROTTLE(1, "Looping");
    }
  } else {
    ros::spin(); // test == "forever" and default
  }
}

TestResult LatticePlannerNode::run_performance_test(Scenario &scenario,
                                                    double planning_time,
                                                    double replanning_time) {
  clear_obstacles();
  for (Obstacle &obstacle : scenario.obstacles) {
    add_obstacle(obstacle, false);

    // Add static obstacles to the obstacle motion planner
    if (obstacle.state == 0) {
      object_lattice_planner_.add_obstacle(obstacle);
    }
  }
  lattice_planner_.set_allowed_volume(scenario.limits);
  geometry_msgs::Point start = scenario.start;
  geometry_msgs::Point goal = scenario.goal;

  initialize_obstacles();

  // Update dynamic objects
  update_obstacles(ros::Time::now().toSec());

  // Observe objects (Fake)
  observe_obstacles(ros::Time::now().toSec());

  // Visualize objects
  // lattice_planner_node->publish_visualization_obstacles();

  // Simulate goal reading (GOAL POSITION)
  geometry_msgs::PoseStamped *goal_position = new geometry_msgs::PoseStamped();
  goal_position->header.stamp = ros::Time::now();
  goal_position->pose.position = goal;
  geometry_msgs::PoseStamped::ConstPtr goal_ptr(goal_position);
  goalCallback(goal_ptr);

  // Simulate position reading (STARTING POSITION)
  geometry_msgs::PointStamped *local_position =
      new geometry_msgs::PointStamped();
  local_position->header.stamp = ros::Time::now();
  local_position->point = start;
  geometry_msgs::PointStamped::ConstPtr local_position_ptr(local_position);
  poseCallback(local_position_ptr);

  // Simulate velocity reading
  geometry_msgs::Vector3Stamped *velocity = new geometry_msgs::Vector3Stamped();
  velocity->header.stamp = ros::Time::now();
  geometry_msgs::Vector3Stamped::ConstPtr velocity_ptr(velocity);
  velCallback(velocity_ptr);

  // ----------

  // Wait for subscribers to connect (For visualization)
  // ros::spinOnce();
  // ros::Rate inv_sleep_time(1);
  // inv_sleep_time.sleep();
  // ros::spinOnce();

  // Plan
  planning_cycle(planning_time, replanning_time);

  // Visualize updated objects
  // lattice_planner_node->publish_visualization_obstacles();

  // lattice_planner_node->publish_visualization_closed_set();

  // -- Create Test Result --
  TestResult result;
  result.scenario = scenario;
  result.planning_time = planning_time;
  result.replanning_time = replanning_time;
  result.profiler = profiler;
  std::map<std::string, double> marks = result.profiler.get_marks_map();
  result.search_time1 = marks["Graph search 1"];
  result.search_time2 = marks["Graph search 2"];
  result.total_time = result.profiler.total();
  result.plan_size = current_plan_.size();
  result.trajectory_size = current_trajectory_.size();
  result.cost = result.plan_size == 0
                    ? 0
                    : lattice_planner_.clone_solution_path().back().cost_f;
  result.primitives_size = lattice_planner_.get_primitives().size();
  result.open_set_size = lattice_planner_.get_frontier().size();
  result.closed_set_size = lattice_planner_.get_closed_set()->size();

  return result;
}

void apply_scenario(LatticePlannerNode *lattice_planner_node, Scenario &scenario) {

  lattice_planner_node->clear_obstacles();
  for (Obstacle &obstacle : scenario.obstacles) {
    lattice_planner_node->add_obstacle(obstacle, false);

    // Add static obstacles to the obstacle motion planner
    if (obstacle.state == 0) {
      lattice_planner_node->object_lattice_planner_.add_obstacle(obstacle);
    }
  }
  lattice_planner_node->lattice_planner_.set_allowed_volume(scenario.limits);
  geometry_msgs::Point start = scenario.start;
  geometry_msgs::Point goal = scenario.goal;

  lattice_planner_node->initialize_obstacles();

  // Update dynamic objects
  lattice_planner_node->update_obstacles(ros::Time::now().toSec());

  // Observe objects (Fake)
  lattice_planner_node->observe_obstacles(ros::Time::now().toSec());

  // Visualize objects
  lattice_planner_node->publish_visualization_obstacles();

  // Simulate position reading (STARTING POSITION)
  geometry_msgs::PointStamped *local_position =
      new geometry_msgs::PointStamped();
  local_position->header.stamp = ros::Time::now();
  local_position->point = start;
  geometry_msgs::PointStamped::ConstPtr local_position_ptr(local_position);
  lattice_planner_node->poseCallback(local_position_ptr);

  // Simulate goal reading (GOAL POSITION)
  geometry_msgs::PoseStamped *goal_position = new geometry_msgs::PoseStamped();
  goal_position->header.stamp = ros::Time::now();
  goal_position->pose.position = goal;
  geometry_msgs::PoseStamped::ConstPtr goal_ptr(goal_position);
  lattice_planner_node->goalCallback(goal_ptr);

  // Simulate velocity reading
  geometry_msgs::Vector3Stamped *velocity = new geometry_msgs::Vector3Stamped();
  velocity->header.stamp = ros::Time::now();
  geometry_msgs::Vector3Stamped::ConstPtr velocity_ptr(velocity);
  lattice_planner_node->velCallback(velocity_ptr);

  // Check if new positions are reciewed (simulator is running) and if so, move
  // the drone to its starting location before returning.

  // Sleep 0.5 seconds
  ros::Rate loop_rate(2);
  loop_rate.sleep();
  ros::spinOnce(); // Spin once to update this->dji_state_
  SearchState start_state = SearchState(start.x, start.y, start.z);
  
  double start_dist = (start_state.position - lattice_planner_node->dji_state_.position).norm();

  ROS_INFO_STREAM_THROTTLE(1, "start_dist: " << start_dist << " current_state: " << lattice_planner_node->dji_state_.position << " start_state: " << start_state.position);

  if ((start_state.position - lattice_planner_node->dji_state_.position).norm() > lattice_planner_node->location_tolerance_) 
  {
    ROS_INFO_STREAM("Preparing for scenario: Moving DJI100 to starting location...");
    lattice_planner_node->fly_to_blocking(start);
    ROS_INFO_STREAM("Preparation complete!");
    ROS_INFO_STREAM("Current location_tolerance: " << lattice_planner_node->location_tolerance_);
  }
  else
  {
    ROS_INFO_STREAM("DJI100 close enough to starting state!");
    ROS_INFO_STREAM("Current location_tolerance: " << lattice_planner_node->location_tolerance_);
  }

  lattice_planner_node->logger.log("scenario_nObstacles", 0.0, lattice_planner_node->object_lattice_planner_.get_obstacles().size());

  // Send goal position to LatticePlanner
  lattice_planner_node->nav_goal_pub_.publish(*goal_position);

}

void LatticePlannerNode::fly_to_blocking(geometry_msgs::Point position) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position = position;
  msg.pose.orientation.w = 1.0;
  pose_pub_.publish(msg);

  SearchState goal_state = SearchState(position.x, position.y, position.z);

  ros::Rate loop_rate(10);
  while ((goal_state.position - this->dji_state_.position).norm() >
         lattice_planner_node->location_tolerance_) {
    loop_rate.sleep();
    ros::spinOnce(); // Spin once to update this->dji_state_
  }
}

void LatticePlannerNode::teleport(geometry_msgs::Point position) {
  ros::Rate loop_rate(5);
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position = position;
  msg.pose.orientation.w = 1.0;
  pose_pub_.publish(msg);
  loop_rate.sleep();
  teleport_pub_.publish(msg);

  SearchState goal_state = SearchState(position.x, position.y, position.z);
  ros::spinOnce();
  if ((goal_state.position - this->dji_state_.position).norm() >
      lattice_planner_node->location_tolerance_) {
      ROS_ERROR_STREAM("Teleportation failed (timed out). Switching to fly_to_blocking(...)...");
      fly_to_blocking(position);
      return;
  }
}

bool LatticePlannerNode::plan_in_collision(
    std::vector<TrajectoryState> current_trajectory, double duration) {
  for (int n = 0; n < current_trajectory.size(); n++) {
    current_trajectory[n].time -= duration;
  }
  return this->lattice_planner_.environment.in_collision(
      lattice_planner_.uav_hitbox, current_trajectory, 0.0);
}

bool LatticePlannerNode::plan_in_collision(
    std::vector<TrajectoryState> current_trajectory, double duration,
    int secondary_traj_start_index) {

  for (int n = 0; n < current_trajectory.size(); n++) {
    current_trajectory[n].time -= duration;
  }

// std::cerr << "secondary_traj_start_index: " << secondary_traj_start_index <<
// ", current_trajectory.size(): " << current_trajectory.size() << " \n";
std:
  vector<TrajectoryState> dynamic_part_current_traj(
      current_trajectory.begin(),
      current_trajectory.begin() + secondary_traj_start_index);
  // std::cerr << "dynamic_part_current_traj.size(): " <<
  // dynamic_part_current_traj.size() << " \n";
  return this->lattice_planner_.environment.in_collision(
      lattice_planner_.uav_hitbox, dynamic_part_current_traj, 0.0);
}

double
LatticePlannerNode::calculate_plan_cost(std::vector<State> &previous_plan,
                                        double duration) {
  if (previous_plan.empty()) {
    return 0.0;
  }
  int n = 0;
  while (n < previous_plan.size() && previous_plan[n].time < duration) {
    n++;
  }
  if (n == 0) {
    return 0.0; // The plan ends before the time ("duration")
  }
  return previous_plan.back().cost_f - previous_plan[n - 1].cost_g;
  ;
}

int LatticePlannerNode::calculate_secondary_plan_index(
    std::vector<State> &current_plan) {
  int n = 1; // TODO First state in current_plan have actionID == -1. It is the
             // root state. But where is it added to the plan!?
  while (n < current_plan.size() &&
         lattice_planner_.get_primitive(current_plan[n].actionID).type !=
             MotionPrimitive::SECONDARY) {
    n++;
  }
  return n;
}

int LatticePlannerNode::calculate_secondary_trajectory_start_index(
    std::vector<TrajectoryState> &current_trajectory,
    std::vector<State> &current_plan, int secondary_plan_start_index) {
  int secondary_start_index = 0;

  int n = current_plan.size() - 1;

  int size_of_current_traj = current_trajectory.size();

  while (n >= secondary_plan_start_index) {
    // std::cerr << "size of prim #" << n << " is: " <<
    // lattice_planner_.get_primitive(current_plan[n].actionID).trajectory.size()
    // << std::endl;
    secondary_start_index +=
        lattice_planner_.get_primitive(current_plan[n].actionID)
            .trajectory.size() -
        1;
    n--;
  }

  std::cerr << "Size of current traj: " << size_of_current_traj << std::endl;
  std::cerr << "secondary_plan_start_index: " << secondary_plan_start_index
            << std::endl;
  std::cerr << "Length of secondary traj: " << secondary_start_index
            << std::endl;
  std::cerr << "Secondary traj start index: "
            << size_of_current_traj - secondary_start_index << std::endl;

  // std::cerr << "secondary_start_index: " << secondary_start_index <<
  // std::endl;
  return (size_of_current_traj - secondary_start_index);
}

void LatticePlannerNode::publish_plan(std::vector<State> plan) {}

void log_obstacle_collisions(double time,
                             LatticePlannerNode *lattice_planner_node) {
  // Logging
  // Calculate distance to obstacles
  double minDistance = 100000000.0;
  int minIndex = -1;
  std::vector<Obstacle> &obstacles =
      lattice_planner_node->lattice_planner_.get_dynamic_obstacles();
  SearchState uav_position = lattice_planner_node->round_search_state(
      lattice_planner_node->dji_state_);
  int numberOfCollidingObstacles = 0;
  double minCenterDistance = -1;
  for (int n = 0; n < obstacles.size(); n++) {
    if (obstacles[n].state == 0)
      continue; // Only dynamic objects
    Hitbox &hb = obstacles[n].get_hitbox();
    SearchState obstacle(hb.getX(), hb.getY(), hb.getZ());
    // Distance between UAV center and object center
    double obstacle_distance =
        (obstacle.position - uav_position.position).norm();
    double UAV_radius =
        lattice_planner_node->lattice_planner_.uav_hitbox.getR();
    double obstacle_radius = hb.getR();
    double geometry_distance =
        obstacle_distance - (obstacle_radius + UAV_radius);
    if (geometry_distance < 0) {
      numberOfCollidingObstacles++;
      ROS_WARN_THROTTLE(1, "Obstacle penetration: %4.2fm!", -geometry_distance);
    }
    // Distance between UAV bounding sphere and object bounding sphere
    if (geometry_distance < minDistance) {
      minIndex = n;
      minDistance = geometry_distance;
      minCenterDistance = obstacle_distance;
    }
  }
  if (minIndex >= 0) {
    lattice_planner_node->logger.log("closest obstacle collision-distance",
                                     time, minDistance);
    lattice_planner_node->logger.log("# colliding obstacles", time,
                                     numberOfCollidingObstacles);
    lattice_planner_node->logger.log("closest obstacle velocity", time,
                                     obstacles[minIndex].max_speed);
    lattice_planner_node->logger.log("closest obstacle distance", time,
                                     minCenterDistance);
    lattice_planner_node->logger.log("closest obstacle radius", time,
                                     obstacles[minIndex].get_hitbox().getR());
  }
}


void LatticePlannerNode::evaluate_primitive_execution() {

  ros::Rate loop_rate(2);

  /* Take inspiration from
        void apply_scenario(LatticePlannerNode *lattice_planner_node,
                            Scenario &scenario)

     Fly to a specific position
     (specify position on the lattice, and the UAV will be there when the code completes):

        SearchState start_state = SearchState(x, y, z);
        if ((start_state.position - dji_state_.position)
            .norm() > location_tolerance) {
            ROS_INFO_STREAM("Preparing for scenario: Moving UAV to starting location...");
            lattice_planner_node->fly_to_blocking(start_state);
        }

     Which primitives can be executed after any given primitive:
        PhysicalEnvironment::primary_primitive_to_primitive_map
  */

  /*
      Step 1:
        Try the code above to fly to SearchState(15,15,15). See if it works as it should.
      Step 2:
        Populate a vector with alla plans we want to execute
        (all valid primitive triples).
        Write out how many plans this is, how many primitives
        and the expected minimal execution time (sum all durations).
      Step 3:
        Run all plans in order.
      Step 4:
        Make sure that the abnormality-node collects and segments all runs correctly.
   */


  // SearchState start_state = SearchState(15,15,15);
  // geometry_msgs::Point start = create_point(15,15,15);

  // if ((start_state.position - dji_state_.position)
  //     .norm() > location_tolerance) {
  //   ROS_INFO_STREAM("Preparing for scenario: Moving UAV to starting location...");
  //   fly_to_blocking(start);
  // }

  // Update dynamic objects
  update_obstacles(ros::Time::now().toSec());

  // Observe objects (Fake)
  observe_obstacles(ros::Time::now().toSec());

  has_valid_goal_ = true;

  while (ros::ok()) {
    // ROS_INFO_STREAM("closeness to target: " <<
    // (goal_state_ -
    // dji_state_).length());
    // ROS_INFO_STREAM("target: " << goal_state_);
    // ROS_INFO_STREAM("state: " << dji_state_);

    double time_loop_start = ros::Time::now().toSec(); // TODO: Should we use
                                                       // current time or fixed
                                                       // simulation time step??



    auto current_state = dji_state_;
    auto goal_state = goal_state_;

    //  Reached goal?
    /*
    double goal_dist = (current_state.position - goal_state.position).norm();
    std::cout << "goal_dist: " << goal_dist << " current: " << current_state
              << " goal: " << goal_state << std::endl;
    if (goal_dist < 0.1) { // Give some margin for weird tracking problems
      double current_time = ros::Time::now().toSec();
      SearchState goal = ...;
      setGoalState(goal);
    }*/

    publish_visualization_obstacles();

    //TODO is this line ok?

    if (has_valid_state()) {

      // planning_cycle(planning_time, replanning_time);

      std::vector<MotionPrimitive> motionprimetives = lattice_planner_.environment.get_primitives();
      //
      std::vector<std::vector<int> > allowd_primatives = lattice_planner_.environment.get_primary_primitive_map();
      std::vector<std::vector<int> > allowed_previous_primatives;

      std::map<int,int> count_primatives;

      //poppulate with empty vectors
      for (int p1 = 0; p1 < allowd_primatives.size(); p1++){
        count_primatives[p1] = 0;
        allowed_previous_primatives.push_back(std::vector<int>());
      }
      //poppulate with all primatives possible to go from p1, with p1 as a possible previous

      std::vector<int> possible_next;

      for (int p1 = 0; p1 < allowd_primatives.size(); p1++){
        possible_next = allowd_primatives[p1];
        for (int i = 0; i < possible_next.size(); i++){
          allowed_previous_primatives[possible_next[i]].push_back(p1);
        }
      }


      int maxValue = std::numeric_limits<int>::max()/2-1;
      //environment.set_allowed_volume(-maxValue, maxValue, -maxValue, maxValue, -maxValue, maxValue);
      set_allowed_volume(-maxValue, maxValue, -maxValue, maxValue, -maxValue, maxValue);

      //Generate all possible triples by iterating over all possible primatives as the mid primative.

      std::vector<int> possible_primatives_first;
      std::vector<int> possible_primatives_third;
      std::map<int,std::vector<std::pair<int,int>> > triplets;

      for (int second_primative_id = 1; second_primative_id < allowd_primatives.size(); second_primative_id++){
        possible_primatives_first = allowed_previous_primatives[second_primative_id];
        triplets[second_primative_id]= std::vector<std::pair<int,int>>();

        for (int first_primative_index = 0; first_primative_index < possible_primatives_first.size(); first_primative_index++){
          possible_primatives_third = allowd_primatives[second_primative_id];

          for (int third_primative_index = 0; third_primative_index < possible_primatives_third.size(); third_primative_index++){

            int p1 = possible_primatives_first[first_primative_index];
            int p2 = second_primative_id;
            int p3 = possible_primatives_third[third_primative_index];
            triplets[second_primative_id].push_back(std::make_pair(p1,p3));

          }
        }
      }

      auto rng = std::default_random_engine {1337};
      std::vector<double> z_values{2};
      
      bool use_specified_triplet = 1;
      //std::vector<int> specified_triplet = {8, 1, 57};
      //std::vector<int> specified_triplet = {64, 1, 57};
      std::vector<int> specified_triplet = {25, 3, 19};
      
      for (int second_primative_id = 1; second_primative_id < allowd_primatives.size(); second_primative_id++){
        for (int pair_index = 0; pair_index<triplets[second_primative_id].size();pair_index++){
          if (pair_index >= 10) continue;
          std::shuffle(std::begin(triplets[second_primative_id]), std::end(triplets[second_primative_id]), rng);
          std::pair<int,int> pair = triplets[second_primative_id][pair_index];
          
          int p1 = pair.first;
          int p2 = second_primative_id;
          int p3 = pair.second;
          count_primatives[second_primative_id] = count_primatives[second_primative_id]+1;
          
          if (use_specified_triplet) {
            // Override p1,p2,p3
            p1 = specified_triplet[0];
            p2 = specified_triplet[1];
            p3 = specified_triplet[2];
          }


          ROS_WARN_STREAM("primative id's are:" << p1 << " " << p2 << " " << p3);

          // ros::spinOnce();
          // //fly back to a start position
          // SearchState start_state = SearchState(5,5,5);
          // geometry_msgs::Point start = create_point(5,5,5);
          // if ((start_state.position - dji_state_.position) .norm() > location_tolerance) {
          //   ROS_WARN_STREAM("Preparing for scenario: Moving UAV to starting location...");
          //   fly_to_blocking(start);
          //   ROS_WARN_STREAM("Done moving UAV");
          // }
          
          // Teleport back to a start position
          geometry_msgs::Point start = create_point(5,5,5);
          ros::spinOnce();
          ros::Duration(1).sleep(); // Wait a second before teleporting
          ROS_WARN_STREAM("UAV position initiating teleportation. Stand By.");
          teleport(start);
          ROS_WARN_STREAM("UAV position resetted (teleportation).");
          ros::spinOnce();

          MotionPrimitive one = motionprimetives[p1];

          MotionPrimitive two = motionprimetives[p2];

          MotionPrimitive three = motionprimetives[p3];
          z_values.push_back(z_values.back()+one.to.position.z()+three.to.position.z()+two.to.position.z() );

          vector<search::SearchState<UAV::SearchState>* > motionprimatives;
          // double delay = 0.1;
          double delay = 3.2;
          //double delay = 2.5;

          search::SearchState<UAV::SearchState> stateone;
          UAV::SearchState inital_state = round_search_state(dji_state_);
          stateone.state.position = Eigen::Vector3f(0,0,0);
          stateone.state.position += inital_state.position;
          stateone.time = 0 + delay;

          search::SearchState<UAV::SearchState> statetwo = lattice_planner_.create_new_state(stateone,p1);

          search::SearchState<UAV::SearchState> statethree = lattice_planner_.create_new_state(statetwo,p2);

          search::SearchState<UAV::SearchState> statefour = lattice_planner_.create_new_state(statethree,p3);

          motionprimatives.push_back(&stateone);
          motionprimatives.push_back(&statetwo);
          motionprimatives.push_back(&statethree);
          motionprimatives.push_back(&statefour);

          lattice_planner_.set_solution_path(motionprimatives);
          lattice_planner_.generate_solution_trajectory(current_trajectory_);

          this->current_plan_ = lattice_planner_.clone_solution_path();

          generate_global_trajectory();
          generate_global_plan();

          ros::Time current_time = ros::Time::now();
          
          Dji_reference_plan_.header.stamp = current_time;
          trajectory_id_pub_.publish(Dji_reference_plan_);

          Dji_reference_trajectory_.header.stamp = current_time;
          trajectory_pub_.publish(Dji_reference_trajectory_);
          publish_visualization_plan_trajectory(Dji_reference_trajectory_,-1);
          
          ros::Duration(statefour.time).sleep();

          // CLEAR by sending a empty plan

          current_time = ros::Time::now();

          Dji_reference_trajectory_ = trajectory_msgs::MultiDOFJointTrajectory();
          Dji_reference_trajectory_.header.stamp = current_time;
          trajectory_pub_.publish(Dji_reference_trajectory_);
          publish_visualization_plan_trajectory(Dji_reference_trajectory_,-1);


          Dji_reference_plan_.header.stamp = current_time;
          Dji_reference_plan_.primitive_id.clear();
          Dji_reference_plan_.time_from_start.clear();
          Dji_reference_plan_.durations.clear();

          trajectory_id_pub_.publish(Dji_reference_plan_);

          ros::spinOnce();
          ROS_INFO_THROTTLE(1, "Looping");

        }
      }

      if ( true ){
        ROS_INFO_STREAM("time is :" << time);
      }

      ROS_INFO_STREAM("Primatives are done" << 1);
      ROS_WARN_STREAM("Done");


      // fly_to_blocking(start);

      new_nav_goal_ = false;
      has_valid_goal_ = false;

      double time_loop_end = ros::Time::now().toSec();
      double loop_time = time_loop_end - time_loop_start;

      // ROS_INFO("Planning time %4.2fs vs intended %4.2fs", loop_time,
      //          planning_time);

      log_obstacle_collisions(ros::Time::now().toSec(), lattice_planner_node);

    } else {
      ROS_WARN_STREAM_THROTTLE(
          1,
          "The planner is missing valid pose or valid velocity. Cannot plan.");
    }
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    ROS_INFO_THROTTLE(1, "Looping");
  }
}
