#include <rrtplanner/rrt.h>


namespace aeplanner_ns
{

Rrt::Rrt(const ros::NodeHandle& nh)
  : nh_(nh)
  , frame_id_("world")
  , path_pub_(nh_.advertise<visualization_msgs::Marker>("rrt_path", 1000))
  , pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1))
  , trajectory_pub_(nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1))
  , octomap_sub_(nh_.subscribe("octomap_full", 1, &Rrt::octomapCallback, this))
  , dynamic_mode_(false)
  , safe_path_srv_(nh_.advertiseService("/rrtplanner/safe_path", &Rrt::safePathSrvCallback, this))
  , pose_sub_(nh_.subscribe("dji_sdk/local_position", 1, &Rrt::poseCallback, this))
  , vel_sub_(nh_.subscribe("dji_sdk/velocity", 1, &Rrt::velCallback, this))
  , goal_sub_(nh_.subscribe("nav_goal", 1, &Rrt::goalCallback, this))
  , obstacles_sub_(nh_.subscribe("obstacles", 1, &Rrt::obstacleCallback, this))
  , scenario_sub_(nh_.subscribe("scenario_info", 1, &Rrt::scenarioCallback, this))
  , ground_truth_dynamic_objects_pub_(nh_.advertise<visualization_msgs::Marker>("rrt_dynamic_objects_ground_truth", 1000))
  , pred_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("predicted_trajectory", 1000))
  , covariance_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("cov_ellipse", 1000))
  , reached_goal_pub_(nh_.advertise<std_msgs::Bool>("/dji0/reached_goal_planner", 1))
  , as_(nh_, "rrt", boost::bind(&Rrt::execute, this, _1), false)
{

  std::string ns = ros::this_node::getNamespace();
  
  //RRT*
  bounding_radius_ = 0.5; //Used in collisionLine
  bounding_overshoot_ = 0.5; //Used in collisionLine
  extension_range_ = 2; //Maximum length of each arrow (edge) in RRT*-tree
  min_nodes_ = 200; //Minimum number of nodes to evaluate
  location_tolerance_ = 2; //location tolerance to goal location
  dji_safety_radius_ = 1.0;
  
  //Kalman Filter / Collisions
  dt_ = 0.5;
  KFiterations_ = 15;
  drone_linear_velocity_ = 1.00347; // m/s Based on experiment
  drone_angular_velocity_ = 1;
  
  std::vector<double> min {-20, -20, -20};
  std::vector<double> max {20, 20, 20};

  boundary_min_ = min;
  boundary_max_ = max;
   if (!ros::param::get("/dji0/rrtplanner_node/dji_safety_radius", dji_safety_radius_)){
    ROS_WARN_STREAM("No dji_safety_radius_ specified. Default: " << dji_safety_radius_);
    abort();
  }

  if (!ros::param::get(ns + "/rrt/min_nodes", min_nodes_)){
    ROS_WARN_STREAM("No minimum nodes specified. Default: " << min_nodes_);
  }

  if (!ros::param::get(ns + "/system/bbx/r", bounding_radius_)){
    ROS_WARN_STREAM("No bounding radius specified. Default: " << bounding_radius_);
  }

  if (!ros::param::get(ns + "/system/bbx/overshoot", bounding_overshoot_))
  {
    ROS_WARN_STREAM("No overshoot paramerer specified. Default: " << bounding_overshoot_);
  }
  
  if (!ros::param::get(ns + "/aep/tree/extension_range", extension_range_))
    ROS_WARN_STREAM("No extension range specified, Default: " << extension_range_);
  if (!ros::param::get(ns + "/boundary/min", boundary_min_))
    ROS_WARN_STREAM("No boundary min specified. Defaulting: " << boundary_min_[0] << ", " << boundary_min_[1] << ", " << boundary_min_[2]);
  if (!ros::param::get(ns + "/boundary/max", boundary_max_))
    ROS_WARN_STREAM("No boundary max specified. Defaulting: " << boundary_max_[0] << ", " << boundary_max_[1] << ", " << boundary_max_[2]);
  
  if (!ros::param::get(ns + "/daep/kf/time_step", dt_)) {
  ROS_WARN_STREAM("No /daep/kf/time_step specified. Default: " << dt_);
  }
  if (!ros::param::get(ns + "/daep/kf/iterations", KFiterations_)) {
    ROS_WARN_STREAM("No /daep/kf/iterations specified. Default: " << KFiterations_);
  }
  if (!ros::param::get(ns + "/drone_linear_velocity", drone_linear_velocity_)) {
    ROS_WARN_STREAM("No /drone_linear_velocity specified. Default: " << drone_linear_velocity_);
  }
  if (!ros::param::get(ns + "/drone_angular_velocity", drone_angular_velocity_)) {
    ROS_WARN_STREAM("No /drone_angular_velocity specified. Default: " << drone_angular_velocity_);
  }

  if (!nh_.getParam("/dji0/rrtplanner_node/location_tolerance", location_tolerance_)) 
  {
    ROS_ERROR_STREAM("Using default location_tolerance: " << location_tolerance_);
    abort();
  }

  /*
   if (!ros::param::get(ns + "/location_tolerance", location_tolerance_)) {
    ROS_WARN_STREAM("No /location_tolerance specified. Default: " << location_tolerance_);
    abort();
  }
  */
 
  ot_ = std::make_shared<octomap::OcTree>(
      1);  // Create dummy OcTree to prevent crash due to ot_ tree not initialized
  as_.start();
}

void Rrt::poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  this->dji_state_.position.x() = msg->point.x;
  this->dji_state_.position.y() = msg->point.y;
  this->dji_state_.position.z() = msg->point.z;
  has_valid_pose_ = true;
  getGoalReachedPub().publish(reached_goal_msg);
}

void Rrt::velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  this->dji_state_.velocity.x() = msg->vector.x;
  this->dji_state_.velocity.y() = msg->vector.y;
  this->dji_state_.velocity.z() = msg->vector.z;
  has_valid_vel_ = true;
}

void Rrt::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

void Rrt::setGoalState(SearchState goal_state) 
{
  this->goal_state_ = goal_state;
  ROS_INFO_STREAM("Goal Updated to: x=" << goal_state.position.x() << " y= "<< goal_state.position.y() << " z= " <<   goal_state.position.z());
  new_nav_goal_ = true;
  has_valid_goal_ = true;
}

void Rrt::obstacleCallback(const rrtplanner::Obstacles::ConstPtr& msg)
{
  for(size_t i = 0; i < msg->obstacles.size(); i++)
  {
    if (msg->obstacles[i].name.find("constantVelocity") != std::string::npos ||
       msg->obstacles[i].name.find("human") != std::string::npos ||
       msg->obstacles[i].name.find("uav") != std::string::npos
       ) 
    {
            dynamic_mode_ = true;
            std::string model_name = msg->obstacles[i].name;
            geometry_msgs::Pose pose;
            pose.position.x = msg->obstacles[i].position.x;
            pose.position.y = msg->obstacles[i].position.y;
            pose.position.z = msg->obstacles[i].position.z;

            geometry_msgs::Twist twist = msg->obstacles[i].state_space_model_velocity;
            
            std::string id = std::to_string(msg->obstacles[i].id);
            dynamic_objects[id] = std::make_pair(pose, twist);
            visualizeGroundTruth(ground_truth_dynamic_objects_pub_, msg->obstacles[i].id, pose, msg->obstacles[i].radius);

    }
    else
    {
      std::string id = std::to_string(msg->obstacles[i].id);
      static_objects[id] = msg->obstacles[i];
    }
  }
}

/**
 * CURRENTLY NOT USED! see obstacleCallback()
 * Subscribe on the gazebo/model_states topic to extract the positions of the 
 * dynamic obstacles. Add these to the dynamic_objects std::map and visualzie 
 * the ground truth in RViz.
*/
void Rrt::updateHumanPositions(const gazebo_msgs::ModelStates& model_states) 
{
    ROS_ERROR("SHOULD NOT BE USED!");
    int human_id = 0;
    for (size_t i = 0; i < model_states.name.size(); ++i) {
        if (model_states.name[i].find("person_walking") != std::string::npos) {
            dynamic_mode_ = true;
            std::string model_name = model_states.name[i];
            geometry_msgs::Pose person_pose = model_states.pose[i];
            geometry_msgs::Twist person_twist = model_states.twist[i];
            dynamic_objects[model_name] = std::make_pair(person_pose, person_twist);
        }
        human_id++;
    }
} 

void Rrt::scenarioCallback(const rrtplanner::Scenario::ConstPtr& msg)
{
  std::vector<double> min {msg->limits.minX, msg->limits.minY, msg->limits.minZ};
  std::vector<double> max = {msg->limits.maxX, msg->limits.maxY, msg->limits.maxZ};
  this->boundary_min_ = min;
  this->boundary_max_ = max;
  has_valid_bounds_ = true;
}


bool Rrt::safePath(nav_msgs::Path path, std::vector<double> time_steps)
{
  auto predicted_data = KFpredictTrajectories();
  
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  visualizePrediction(pred_marker_pub_, covariance_marker_pub_, trajectories, all_ellipses);

  for(int i = 0; i < path.poses.size(); i++)
  {

    if(i == path.poses.size() - 1)
    {
      //Avoid to index outside of path
      break;
    }

    if(isCollision(path.poses[i], path.poses[i + 1], time_steps[i], predicted_data))
    {
      return false;
    }
  }
  return true;
}


bool Rrt::safePathSrvCallback(rrtplanner::SafePath::Request& request, rrtplanner::SafePath::Response& response)
{

  auto predicted_data = KFpredictTrajectories();

  for(int i = 0; i < request.path.poses.size(); i++)
  {

    if(i == request.path.poses.size() - 1)
    {
      //Avoid to index outside of path
      break;
    }

    if(isCollision(request.path.poses[i], request.path.poses[i + 1], request.time_steps[i], predicted_data))
    {
      response.safe = false;
      return true;
    }
  }

  response.safe = true;
  return true;
}


bool Rrt::isCollision(const geometry_msgs::PoseStamped& posestamped_parent, const geometry_msgs::PoseStamped& posestamped, double time_of_arrival,
                      std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data)
{

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  //Extract each person's trajectory and covariance trajectory
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  Eigen::Vector3d next_pose(posestamped.pose.position.x, 
                            posestamped.pose.position.y, 
                            posestamped.pose.position.z);

  Eigen::Vector3d next_pose_parent(posestamped_parent.pose.position.x, 
                                   posestamped_parent.pose.position.y, 
                                   posestamped_parent.pose.position.z);

  bool collision = checkCollision(time_of_arrival, next_pose, 
                                  trajectories, all_ellipses, next_pose_parent);

  return collision;
}

/**
* This function uses the Kalman Filter with the Constant Velocity
* motion model to predict the future trajectory of each dynamic obstacle.
* The returned data consists of the means and covariances for each dynamic obstacle in a trajectory.
*/
std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> Rrt::KFpredictTrajectories()
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian, we need to build a new Kalman filter
  int n = 4; //Number of states
  int m = 2; //Number of states we measure

  //Matrices for the Kalman filter
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant Velocity Model
  A <<  1, 0, dt_, 0, 
        0, 1, 0, dt_, 
        0, 0, 1, 0, 
        0, 0, 0, 1;

  C << 1, 0, 0, 0, 0, 1, 0, 0;

  //Reasonable covariance matrices
  Q << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  R << 0.01, 0,
       0, 0.01;
  
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1; 

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  KalmanFilter kf;

  double t = 0;
  int safety_iteration = 0; // Bounding covariance (1 second)
  
  for (const auto& dynamic_obstacle : dynamic_objects) {
      const std::string& key = dynamic_obstacle.first;
      const geometry_msgs::Pose& pose = dynamic_obstacle.second.first;
      const geometry_msgs::Twist& twist = dynamic_obstacle.second.second;

      // Create a new Kalman filter for each dynamic obstacle
      std::vector<double> xcoords{};
      std::vector<double> ycoords{};
      std::vector<double> zcoords{};
      std::vector<Eigen::MatrixXd> covariance_matrices(KFiterations_);

      double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
      double vx = twist.linear.x, vy = twist.linear.y;

      // Initalize kalman filter for the first time or from memory.
      if(kalman_filters.count(key) > 0)
      {
        kf = kalman_filters[key];
      }
      else
      {
        kf = KalmanFilter(dt_, A, C, Q, R, P);
        x0 << xcoord, ycoord, vx, vy;
        P0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; 
        kf.init(t, x0, P0);
      }

      // Perform one round of the Kalman filter
      kf.predict();
      y << xcoord, ycoord;
      kf.update(y);

      kalman_filters[key] = kf; //Save current state of xk and Pk

      // Create the trajectory for the current dynamic obstacle
      Eigen::MatrixXd safety_margin(n, n);
      
      for (int i = 0; i < KFiterations_; i++)
      {
        kf.predict();
        xcoords.push_back(kf.state()(0));
        ycoords.push_back(kf.state()(1));
        zcoords.push_back(zcoord);
        if(i == safety_iteration)
        {
          //Because of controllability we can only extract one of these
          //ellipses. See paper for explanation.
          safety_margin = kf.covariance();
        }
      }

      std::fill(covariance_matrices.begin(), covariance_matrices.end(), safety_margin);  
      kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices)); 
  }
    return kf_data;
}

void Rrt::planning_cycle(int min_nodes)
{

  ros::Time start = ros::Time::now();
  long eval_nodes{};

  //Create correct msg for planner
  rrtplanner::rrtGoal rrt_goal;
  rrt_goal.start.header.stamp = ros::Time::now();
  rrt_goal.start.header.frame_id = "world";

  //Collect starting point
  geometry_msgs::PoseStamped ps;
  ps.pose.position.x = dji_state_.position.x();
  ps.pose.position.y = dji_state_.position.y();
  ps.pose.position.z = dji_state_.position.z();
  rrt_goal.start = ps;

  //Insert goal(s)
  geometry_msgs::Pose goal;
  goal.position.x = goal_state_.position.x(), 
  goal.position.y = goal_state_.position.y(), 
  goal.position.z = goal_state_.position.z();
  rrt_goal.goal_poses.poses.push_back(goal);

  rrtplanner::rrtResult result;
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    return;
  }
  if (!rrt_goal.goal_poses.poses.size())
  {
    ROS_WARN("No goals received");
    return;
  }

  //Get predictions of dynamic obstacles
  auto predicted_data = KFpredictTrajectories();
  
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

   for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  visualizePrediction(pred_marker_pub_, covariance_marker_pub_, trajectories, all_ellipses);

  std::vector<RrtNode*> found_goals;

  kdtree* kd_tree = kd_create(3);    // Initalize tree
  kdtree* goal_tree = kd_create(3);  // kd tree with all goals
  for (int i = 0; i < rrt_goal.goal_poses.poses.size(); ++i)
  {
    Eigen::Vector3d* g = new Eigen::Vector3d(rrt_goal.goal_poses.poses[i].position.x,
                                             rrt_goal.goal_poses.poses[i].position.y,
                                             rrt_goal.goal_poses.poses[i].position.z);
    kd_insert3(goal_tree, (*g)[0], (*g)[1], (*g)[2], g);
  }

  // Initialize root position
  RrtNode* root = new RrtNode;
  root->pos[0] = rrt_goal.start.pose.position.x;
  root->pos[1] = rrt_goal.start.pose.position.y;
  root->pos[2] = rrt_goal.start.pose.position.z;
  root->parent = NULL;
  kd_insert3(kd_tree, root->pos[0], root->pos[1], root->pos[2], root);

  
  for (int i = 0; i < min_nodes; ++i)
  {

    // Sample new position
    Eigen::Vector3d sampled_node = sample();
    eval_nodes++;

    // Get nearest neighbour
    RrtNode* nearest = chooseParent(kd_tree, sampled_node, extension_range_);
    if (!nearest)
      continue;

    // Calculate position for new node
    Eigen::Vector3d normalised_node = getNewPosNormalized(sampled_node, nearest->pos, extension_range_);
    Eigen::Vector3d direction = normalised_node - nearest->pos;
    
    //###################### DAEP ########################

    // Estimate time to reach node
    double time_to_reach_node = nearest->time_cost(drone_linear_velocity_, drone_angular_velocity_) + nearest->time_to_reach(normalised_node, drone_linear_velocity_, drone_angular_velocity_);

    // Check if the sampled node will collide in the future with a dynamic obstacle
    bool dynamic_collision = checkCollision(time_to_reach_node, normalised_node, trajectories, all_ellipses, nearest->pos);

    //SDMP
    bool static_collision = checkIntersectStaticObstacle(nearest->pos, normalised_node, bounding_radius_);

    //###################### END DAEP #####################

    if(!dynamic_collision and !static_collision)
    {
      // Add node to tree
      RrtNode* valid_node = addNodeToTree(kd_tree, nearest, normalised_node);
     
      rewire(kd_tree, valid_node, extension_range_, bounding_radius_, bounding_overshoot_, time_to_reach_node, trajectories, all_ellipses);
     
      visualizeEdge(valid_node, i);
     
      // Check if goal has been reached
      RrtNode* tmp_goal = getGoal(goal_tree, valid_node, extension_range_, bounding_radius_, bounding_overshoot_);
      
      if (tmp_goal)
      {
        found_goals.push_back(tmp_goal);
      }
    
    }
    else
    {
      --i;
    }
  }

  std::pair<nav_msgs::Path, std::vector<double>> path_and_time = getBestPath(found_goals);

  this->plan_ = path_and_time.first;
  this->time_steps_ = path_and_time.second;

  this->totalTime_ = ros::Time::now() - start;
  this->planSize_ = this->plan_.poses.size();
  this->evalStates_ = eval_nodes;
  this->foundPlan_ = !found_goals.empty();
  
  delete root;
  kd_free(kd_tree);
  kd_free(goal_tree);
}


void Rrt::execute(const rrtplanner::rrtGoalConstPtr& goal)
{
  rrtplanner::rrtResult result;
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }
  if (!goal->goal_poses.poses.size())
  {
    ROS_WARN("No goals received");
    as_.setSucceeded(result);
    return;
  }

  //Get predictions of dynamic obstacles
  auto predicted_data = KFpredictTrajectories();


  
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

   for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  double l = extension_range_;
  double r = bounding_radius_;
  double r_os = bounding_overshoot_;
  std::vector<RrtNode*> found_goals;

  kdtree* kd_tree = kd_create(3);    // Initalize tree
  kdtree* goal_tree = kd_create(3);  // kd tree with all goals
  for (int i = 0; i < goal->goal_poses.poses.size(); ++i)
  {
    Eigen::Vector3d* g = new Eigen::Vector3d(goal->goal_poses.poses[i].position.x,
                                             goal->goal_poses.poses[i].position.y,
                                             goal->goal_poses.poses[i].position.z);
    kd_insert3(goal_tree, (*g)[0], (*g)[1], (*g)[2], g);
  }


  // Initialize root position
  RrtNode* root = new RrtNode;
  root->pos[0] = goal->start.pose.position.x;
  root->pos[1] = goal->start.pose.position.y;
  root->pos[2] = goal->start.pose.position.z;
  root->parent = NULL;
  kd_insert3(kd_tree, root->pos[0], root->pos[1], root->pos[2], root);

  
  for (int i = 0; i < min_nodes_; ++i)
  {
    // Sample new position
    Eigen::Vector3d sampled_node = sample();
    // Get nearest neighbour
    RrtNode* nearest = chooseParent(kd_tree, sampled_node, l);
    if (!nearest)
      continue;

    // Calculate position for new node
    Eigen::Vector3d normalised_node = getNewPosNormalized(sampled_node, nearest->pos, l);
    Eigen::Vector3d direction = normalised_node - nearest->pos;
    
  //###################### DAEP ########################
    // Estimate time to reach node
    double time_to_reach_node = nearest->time_cost(drone_linear_velocity_, drone_angular_velocity_) + nearest->time_to_reach(normalised_node,drone_linear_velocity_, drone_angular_velocity_);

    // Check if the sampled node will collide in the future
    bool collision = checkCollision(time_to_reach_node, normalised_node, trajectories, all_ellipses, nearest->pos);

  //###################### END DAEP #####################

    if(!collisionLine(nearest->pos, normalised_node + direction.normalized() * r_os, r) and !collision)
    {
      // Add node to tree
      RrtNode* valid_node = addNodeToTree(kd_tree, nearest, normalised_node);
      //rewire(kd_tree, valid_node, l, r, r_os);
      visualizeEdge(valid_node, i);
      // Check if goal has been reached
      RrtNode* tmp_goal = getGoal(goal_tree, valid_node, l, r, r_os);
      if (tmp_goal)
      {
        found_goals.push_back(tmp_goal);
      }
    }
    else
    {
      --i;
    }
  }

  std::pair<nav_msgs::Path, std::vector<double>> path_and_time = getBestPath(found_goals);

  result.path = path_and_time.first;
  result.time_steps = path_and_time.second;

  delete root;
  kd_free(kd_tree);
  kd_free(goal_tree);

  as_.setSucceeded(result);
}


void Rrt::octomapCallback(const octomap_msgs::Octomap& msg)
{
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  if (ot)
    delete ot;
}


Eigen::Vector3d Rrt::sample()
{
  Eigen::Vector3d x_samp;
  for (int i = 0; i < 3; ++i)
  {
    x_samp[i] = boundary_min_[i] + ( (((double)rand()) / ((double)RAND_MAX)) * (boundary_max_[i] - boundary_min_[i]));
  }

  return x_samp;
}


RrtNode* Rrt::chooseParent(kdtree* kd_tree, Eigen::Vector3d node, double extension_range)
{
  kdres* nearest = kd_nearest_range3(kd_tree, node[0], node[1], node[2], extension_range);
  if (kd_res_size(nearest) <= 0)
  {
    nearest = kd_nearest3(kd_tree, node[0], node[1], node[2]);
  }
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RrtNode* node_nn = (RrtNode*)kd_res_item_data(nearest);
  int i = 0;

  RrtNode* best_node = node_nn;
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (best_node and node_nn->cost() < best_node->cost())
      best_node = node_nn;

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}


void Rrt::rewire(kdtree* kd_tree, RrtNode* new_node, double extension_range, double bounding_radius, double bounding_overshoot, double time_to_reach_node,
                  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses)
{
  RrtNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2], extension_range);
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (node_nn->cost() > new_node->cost() + (node_nn->pos - new_node->pos).norm())
    {
      //SDMP
      bool dynamic_collision = checkCollision(time_to_reach_node, new_node->pos, trajectories, all_ellipses, node_nn->pos + (node_nn->pos - new_node->pos).normalized());
      bool static_collision = checkIntersectStaticObstacle(node_nn->pos + (node_nn->pos - new_node->pos).normalized(), new_node->pos, bounding_radius);
      if (!static_collision and !dynamic_collision)
      {
        node_nn->parent = new_node;
      }
    }
    kd_res_next(nearest);
  }
}


Eigen::Vector3d Rrt::getNewPosNormalized(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l)
{
  Eigen::Vector3d direction = sampled - parent;
  if (direction.norm() > l)
    direction = l * direction.normalized();

  return parent + direction;
}


RrtNode* Rrt::addNodeToTree(kdtree* kd_tree, RrtNode* parent,
                            Eigen::Vector3d new_pos)
{
  RrtNode* new_node = new RrtNode;
  new_node->pos = new_pos;

  new_node->parent = parent;
  parent->children.push_back(new_node);
  kd_insert3(kd_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2],
             new_node);

  return new_node;
}


RrtNode* Rrt::getGoal(kdtree* goal_tree, RrtNode* new_node, double extension_range, double bounding_radius, double bounding_overshoot)
{
  kdres* nearest_goal = kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return NULL;
  }
  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < location_tolerance_)
  {
    if (!collisionLine(new_node->pos, *g_nn + (*g_nn - new_node->pos).normalized() * bounding_overshoot, bounding_radius))
    {
      return new_node;
    }
  }
   
  return NULL;
}

double getPathLength(nav_msgs::Path path)
{
  double dist{};

  if(path.poses.empty() || path.poses.size() == 1)
  {
    return dist;
  }

  for(int i = 0; i < path.poses.size()-1; i++)
  {
    Eigen::Vector3d p1 (path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
    Eigen::Vector3d p2 (path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y, path.poses[i+1].pose.position.z);
    Eigen::Vector3d dir = p2 - p1;
    dist += dir.norm();
  }
  return dist;
}


std::pair<nav_msgs::Path, std::vector<double>> Rrt::getBestPath(std::vector<RrtNode*> goals)
{

  std::vector<double> time_steps{};
  nav_msgs::Path path;

  if (goals.size() == 0)
  {
    return std::make_pair(path, time_steps);
  }

  RrtNode* best_node = goals[0];

  for (int i = 0; i < goals.size(); ++i)
    if (best_node->cost() > goals[i]->cost())
      best_node = goals[i];

  RrtNode* n = best_node;

  // Extract data
  this->costToGoal_ = best_node->cost();
  this->timeToReach_ = best_node->time_cost(drone_linear_velocity_, drone_angular_velocity_);


  //Extract path to the best node
  for (int id = 0; n->parent; ++id)
  {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = n->pos[0];
    p.pose.position.y = n->pos[1];
    p.pose.position.z = n->pos[2];
    
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    // Zero out rotation along
    // x and y axis so only
    // yaw is kept
    Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0], n->pos[1] - n->parent->pos[1], 0);
    q.setFromTwoVectors(init, dir);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    path.poses.push_back(p);
    time_steps.push_back(n->time_cost(drone_linear_velocity_, drone_angular_velocity_)); //Add time it takes to reach this node from root
    n = n->parent;
  }

  // Add final node, since when n->parent == NULL 
  // loop breaks, thus we don't add final n to path.
  geometry_msgs::PoseStamped p;
  p.pose.position.x = n->pos[0];
  p.pose.position.y = n->pos[1];
  p.pose.position.z = n->pos[2];
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0; 
  p.pose.orientation.w = 1; 

  path.poses.push_back(p);
  time_steps.push_back(n->time_cost(drone_linear_velocity_, drone_angular_velocity_)); //Add time it takes to reach this node from root

  std::reverse(time_steps.begin(), time_steps.end());
  std::reverse(path.poses.begin(), path.poses.end());

  visualizePath(best_node);
  return std::make_pair(path, time_steps);
}


std::vector<geometry_msgs::Pose> Rrt::checkIfGoalReached(kdtree* goal_tree,
                                                         RrtNode* new_node, double l,
                                                         double r, double r_os)
{
  std::vector<geometry_msgs::Pose> path;

  kdres* nearest_goal =
      kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return path;
  }
  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < 2 * l)
  {
    if (!collisionLine(new_node->pos,
                       *g_nn + (*g_nn - new_node->pos).normalized() * r_os, r))
    {
      RrtNode* n = new_node;
      for (int id = 0; n->parent; ++id)
      {
        geometry_msgs::Pose p;
        p.position.x = n->pos[0];
        p.position.y = n->pos[1];
        p.position.z = n->pos[2];
        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        // Zero out rotation
        // along x and y axis
        // so only yaw is kept
        Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0],
                            n->pos[1] - n->parent->pos[1], 0);
        q.setFromTwoVectors(init, dir);

        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        path.push_back(p);
        visualizePose(p, id);

        n = n->parent;
      }

      visualizePath(new_node);
    }
  }

  return path;
}


bool Rrt::between(Eigen::Vector3d x, Eigen::Vector3d axis, Eigen::Vector3d p1, Eigen::Vector3d p2)
{
  return ( (x.dot(axis) >= axis.dot(p1)) && (x.dot(axis) <= axis.dot(p2)) );
}


bool Rrt::lineInsideBox(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector3d box_center, Eigen::Vector3d box_size)
{
  // Center and radius
  double x = box_center[0];
  double y = box_center[1];
  double z = box_center[2];
  double rx = box_size[0];
  double ry = box_size[1];
  double rz = box_size[2];

  // Edges in box
  Eigen::Vector3d p1(x - rx, y - ry, z + rz);
  Eigen::Vector3d p2(x + rx, y - ry, z + rz);
  Eigen::Vector3d p3(x - rx, y - ry, z - rz);
  Eigen::Vector3d p4(x - rx, y + ry, z + rz);

  // Three perpendicular edges of the box
  Eigen::Vector3d u = p4 - p1;
  Eigen::Vector3d v = p2 - p1;
  Eigen::Vector3d w = p3 - p1;

  bool start_in_box = between(start, u, p1, p4) && between(start, v, p1, p2) && between(start, w, p1, p3);
  bool end_in_box = between(end, u, p1, p4) && between(end, v, p1, p2) && between(end, w, p1, p3);

  return start_in_box && end_in_box;
}


std::vector<Eigen::Vector3d> Rrt::lineIntersectBox(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector3d box_center, Eigen::Vector3d box_size)
{
  Eigen::Vector3d direction = end - start;
  Eigen::Vector3d minPoint = box_center - box_size;
  Eigen::Vector3d maxPoint = box_center + box_size;
  Eigen::Vector3d start_to_min = minPoint - start;
  Eigen::Vector3d start_to_max = maxPoint - start;
  double near = -std::numeric_limits<double>::infinity();
  double far = std::numeric_limits<double>::infinity();
  std::vector<Eigen::Vector3d> intersections{};

  for(int axis = 0; axis < 3; axis++)
  {
    if(direction[axis] == 0)
    {
      if(start_to_min[axis] > 0 || start_to_max[axis] < 0)
      {
        return intersections;
      }
    }
    else
    {
      double t1 = (start_to_min[axis] / direction[axis]);
      double t2 = (start_to_max[axis] / direction[axis]);
      double tMin = std::min(t1, t2);
      double tMax = std::max(t1, t2);
      if (tMin > near) { near = tMin; }
      if (tMax < far) { far = tMax; }
      if (near > far || far < 0) { return intersections; }
    }
  }

  if(near >= 0 && near <= 1)
  {
    Eigen::Vector3d intersection = (start + (direction * near));
    intersections.push_back(intersection);
  }

  if(far >= 0 && far <= 1)
  {
    Eigen::Vector3d intersection = (start + (direction * far));
    intersections.push_back(intersection);
  }

  return intersections;
}

bool Rrt::pointTooCloseToWall(Eigen::Vector3d point, Eigen::Vector3d box_center, Eigen::Vector3d box_size)
{
  Eigen::Vector3d minPoint = box_center - box_size;
  Eigen::Vector3d maxPoint = box_center + box_size;

  float dx = std::max({(minPoint[0] - point[0]), 0.0, (point[0] - maxPoint[0])});
  float dy = std::max({(minPoint[1] - point[1]), 0.0, (point[1] - maxPoint[1])});
  float dz = std::max({(minPoint[2] - point[2]), 0.0, (point[2] - maxPoint[2])});

  float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

  float dji100_diagonal_wheelbase = 0.65; //meter
  float dji100_vertical_base = sin(30.0*PI/180) * dji100_diagonal_wheelbase;
  float extra = 0.1; //Add extra to make the dji100 rotate without hitting the wall.

  /*
  |
  | --- ¤ #Need to keep distance from walls when sampling
  |
  */

  if (distance > (dji100_vertical_base + extra))
  {
    return false;
  }

  return true;
}


bool Rrt::checkIntersectStaticObstacle(Eigen::Vector3d p1, Eigen::Vector3d p2, double bounding_radius)
{

  for (const auto& static_obstacle : static_objects) 
  {
    Eigen::Vector3d box_center(static_obstacle.second.position.x, static_obstacle.second.position.y, static_obstacle.second.position.z);
    Eigen::Vector3d box_size(static_obstacle.second.radius3D.x, static_obstacle.second.radius3D.y, static_obstacle.second.radius3D.z);

    bool tooClose = pointTooCloseToWall(p2, box_center, box_size);

    if(tooClose)
    {
      return true;
    }

    bool inside = lineInsideBox(p1, p2, box_center, box_size);

    if(inside)
    {
      return true;
    }

    std::vector<Eigen::Vector3d> intersections = lineIntersectBox(p1, p2, box_center, box_size);

    if(!intersections.empty())
    {
      return true;
    }
  }

  return false;
}



/**
 * Given two points as Vector3d and a bounding radius, will this funtion check that
 * if you draw a line between these two points with a radius bounding_radius (think big hot dog)
 * this function will return true if there is a collision with any voxel in the octomap and
 * the giant hot dog, otherwise false.
 */
bool Rrt::collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double bounding_radius)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  octomap::point3d min(std::min(p1[0], p2[0]) - bounding_radius, 
                       std::min(p1[1], p2[1]) - bounding_radius,
                       std::min(p1[2], p2[2]) - bounding_radius);

  octomap::point3d max(std::max(p1[0], p2[0]) + bounding_radius, 
                       std::max(p1[1], p2[1]) + bounding_radius,
                       std::max(p1[2], p2[2]) + bounding_radius);

  double lsq = (end - start).norm_sq();
  double rsq = bounding_radius * bounding_radius;

  octomap::point3d query(p2[0], p2[1], p2[2]);
  octomap::OcTreeNode* ot_res = ot->search(query);
    
  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max),
                                          it_end = ot->end_leafs_bbx();
                                          it != it_end; ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    if (it->getLogOdds() > 0)
    {  // Node is occupied
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < bounding_radius)
      {
        return true;
      }
    }
  }
  return false;
}

/**
 * From a vector of covariance matrices, compute a list of tuples that
 * represent each covariance circle.
 * 
 * Return: (major_lenght, minor_length, eigenvectors) 
*/
std::vector<std::tuple<double, double, Eigen::MatrixXd>> Rrt::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> ellipses{};

  for(auto const& cov_matrix : cov_matrices)
  {
    //Extract the part concerning x,y
    Eigen::MatrixXd cov_matrix_xy = cov_matrix.block<2,2>(0,0);

    // Compute the eigenvalues and eigenvectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov_matrix_xy);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

    // Compute the length of the major and minor axes of the ellipse
    double major_length = std::sqrt(std::max(eigenvalues(0), eigenvalues(1)));
    double minor_length = std::sqrt(std::min(eigenvalues(0), eigenvalues(1)));

    ellipses.push_back(std::make_tuple(major_length, minor_length, eigenvectors));
  }
  return ellipses;
}

/* 
* Calculate if there is a potential collision with any moving obstacle in time t.
*/
bool Rrt::checkCollision(double t, 
                          Eigen::Vector3d point, 
                          std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses,
                          Eigen::Vector3d parent)
{
  double max_time_step = KFiterations_ * dt_;
  int N_persons = trajectories.size();
  
  // Index the correct circle
  int covariance_index = getCovarianceIndex(max_time_step, dt_, t);

  if(covariance_index == -1)
  { 
    // Outside our prediction
    return false;
  }

  for(int i = 0; i < N_persons; i++)
  { 
      auto personEllipses = covarianceEllipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];
      double radius = std::get<0>(covarianceEllipse);

      const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
        
      if(lineIntersectsCircle(parent, point, center, radius))
      {
        return true;
      }
  }
  return false;
}

/*
Checks if a line between p1 and p2 in xy-plane intersects a circle in the same plane*/
bool Rrt::lineIntersectsCircle(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                               const Eigen::Vector3d center, double radius)
{
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double a = dx * dx + dy * dy;
  double b = 2 * (dx * (p1.x() - center.x()) + dy * (p1.y() - center.y()));
  double c = center.x() * center.x() + center.y() * center.y();
  c += p1.x() * p1.x() + p1.y() * p1.y();
  c -= 2 * (center.x() * p1.x() + center.y() * p1.y());
  c -= radius * radius;
  double discriminant = b * b - 4 * a * c;

  if(discriminant >= 0)
  {
    double t1 = (-b - sqrt(discriminant)) / (2*a);
    double t2 = (-b + sqrt(discriminant)) / (2*a);
  
    if((t1 >= 0 and t1 <= 1) or (t2 >= 0 and t2 <= 1))
    {
      return true;
    }
  }
  return false;

}

/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int Rrt::getCovarianceIndex(double max_time_step, double time_step, double t){
  
  int num_steps = static_cast<int>(max_time_step / time_step); // Calculate the number of time steps

  if(t > max_time_step)
  {
    //No prediction available
    return -1;
  }

  // Initialize the minimum difference and index
  double min_difference = std::abs(t - time_step);
  int index = 0;

  // Loop through each time step and find the closest one
  for (int i = 1; i < num_steps; ++i) {
    double time = i * time_step; // Calculate the time for the current step
    double difference = std::abs(t - time);
    if (difference < min_difference) {
      min_difference = difference;
      index = i;
    }
  }
  return index;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James -
// gjames@NVIDIA.com Lisc:
// Free code - no warranty &
// no money back.  Use it all
// you want Desc:
//    This function tests if
//    the 3D point 'pt' lies
//    within an arbitrarily
// oriented cylinder.  The
// cylinder is defined by an
// axis from 'pt1' to 'pt2',
// the axis having a length
// squared of 'lsq'
// (pre-compute for each
// cylinder to avoid repeated
// work!), and radius squared
// of 'rsq'.
//    The function tests
//    against the end caps
//    first, which is cheap ->
//    only
// a single dot product to
// test against the parallel
// cylinder caps.  If the
// point is within these, more
// work is done to find the
// distance of the point from
// the cylinder axis.
//    Fancy Math (TM) makes
//    the whole test possible
//    with only two
//    dot-products
// a subtract, and two
// multiplies.  For clarity,
// the 2nd mult is kept as a
// divide.  It might be faster
// to change this to a mult by
// also passing in 1/lengthsq
// and using that instead.
//    Elminiate the first 3
//    subtracts by specifying
//    the cylinder as a base
// point on one end cap and a
// vector to the other end cap
// (pass in {dx,dy,dz} instead
// of 'pt2' ).
//
//    The dot product is
//    constant along a plane
//    perpendicular to a
//    vector. The magnitude of
//    the cross product
//    divided by one vector
//    length is
// constant along a cylinder
// surface defined by the
// other vector as axis.
//
// Return:  -1.0 if point is
// outside the cylinder
// Return:  distance squared
// from cylinder axis if point
// is inside.
//
//-----------------------------------------------------------------------------
float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                        float lsq, float rsq, const octomap::point3d& pt)
{
  float dx, dy,
      dz;  // vector d  from
           // line segment
           // point 1 to point
           // 2
  float pdx, pdy,
      pdz;  // vector pd from
            // point 1 to test
            // point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate
                           // so pt1 is
                           // origin.
                           // Make vector
                           // from
  dy = pt2.y() - pt1.y();  // pt1 to
                           // pt2.  Need
                           // for this
                           // is easily
                           // eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from
                           // pt1 to test
                           // point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors
  // to see if point lies
  // behind the cylinder cap
  // at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero
  // the point is behind the
  // pt1 cap. If greater than
  // the cylinder axis line
  // segment length squared
  // then the point is outside
  // the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
  {
    return (-1.0f);
  }
  else
  {
    // Point lies within the
    // parallel caps, so find
    // distance squared from
    // point to line, using
    // the fact that sin^2 +
    // cos^2 = 1 the dot =
    // cos() * |d||pd|, and
    // cross*cross = sin^2 *
    // |d|^2 * |pd|^2
    // Carefull: '*' means
    // mult for scalars and
    // dotproduct for vectors
    // In short, where dist is
    // pt distance to cyl
    // axis: dist = sin( pd to
    // d ) * |pd| distsq = dsq
    // = (1 - cos^2( pd to d))
    // * |pd|^2 dsq = ( 1 -
    // (pd * d)^2 / (|pd|^2 *
    // |d|^2) ) * |pd|^2 dsq =
    // pd * pd - dot * dot /
    // lengthsq
    //  where lengthsq is d*d
    //  or |d|^2 that is
    //  passed into this
    //  function

    // distance squared to the
    // cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
    {
      return (-1.0f);
    }
    else
    {
      return (dsq);  // return
                     // distance
                     // squared
                     // to
                     // axis
    }
  }
}

void Rrt::visualizeNode(geometry_msgs::Point pos, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "nodes";
  a.type = visualization_msgs::Marker::SPHERE;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position = pos;

  a.scale.x = 0.2;
  a.scale.y = 0.2;
  a.scale.z = 0.2;
  a.color.r = 0.2;
  a.color.g = 0.7;
  a.color.b = 0.2;
  ;
  a.color.a = 1;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;
  path_pub_.publish(a);
}

void Rrt::visualizePose(geometry_msgs::Pose pose, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "pose";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose = pose;
  a.scale.x = 0.4;
  a.scale.y = 0.1;
  a.scale.z = 0.1;
  a.color.r = 1.0;
  a.color.g = 0.0;
  a.color.b = 0.0;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void Rrt::visualizeEdge(RrtNode* node, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "vp_branches";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position.x = node->parent->pos[0];
  a.pose.position.y = node->parent->pos[1];
  a.pose.position.z = node->parent->pos[2];
  Eigen::Quaternion<double> q;
  Eigen::Vector3d init(1.0, 0.0, 0.0);
  Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0],
                      node->pos[1] - node->parent->pos[1],
                      node->pos[2] - node->parent->pos[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  a.pose.orientation.x = q.x();
  a.pose.orientation.y = q.y();
  a.pose.orientation.z = q.z();
  a.pose.orientation.w = q.w();
  a.scale.x = dir.norm();
  a.scale.y = 0.05;
  a.scale.z = 0.05;
  a.color.r = 1;
  a.color.g = 0.3;
  a.color.b = 0.7;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void Rrt::visualizePath(RrtNode* node)
{
  for (int id = 0; node->parent; ++id)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = id;
    a.header.frame_id = frame_id_;
    a.id = id;
    a.ns = "path";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose.position.x = node->parent->pos[0];
    a.pose.position.y = node->parent->pos[1];
    a.pose.position.z = node->parent->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0],
                        node->pos[1] - node->parent->pos[1],
                        node->pos[2] - node->parent->pos[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    a.pose.orientation.x = q.x();
    a.pose.orientation.y = q.y();
    a.pose.orientation.z = q.z();
    a.pose.orientation.w = q.w();
    a.scale.x = dir.norm();
    a.scale.y = 0.07;
    a.scale.z = 0.07;
    a.color.r = 0.7;
    a.color.g = 0.7;
    a.color.b = 0.3;
    a.color.a = 1.0;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;

    path_pub_.publish(a);

    node = node->parent;
  }
}

void Rrt::visualizeGoals(std::vector<geometry_msgs::Pose> goals)
{
  for (int i = 0; i < goals.size(); ++i)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = i;
    a.header.frame_id = frame_id_;
    a.id = i;
    a.ns = "goals";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose = goals[i];

    a.scale.x = 0.2;
    a.scale.y = 0.1;
    a.scale.z = 0.1;
    a.color.r = 1.0;
    a.color.g = 0.3;
    a.color.b = 0.7;
    a.color.a = 1;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;
    path_pub_.publish(a);
  }
}

/**
 * Visualize the ground truth for each dynamic obstacle in RViz as a cylinder.
*/
void Rrt::visualizeGroundTruth(ros::Publisher& marker_pub, const int& object_id, const geometry_msgs::Pose& pose, float radius) 
{
    // Create visualization marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  
    marker.header.stamp = ros::Time::now();
    marker.id = object_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose = pose;
    marker.scale.x = radius * 2; 
    marker.scale.y = radius * 2;  
    marker.scale.z = radius * 2;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;  
    marker.color.g = 1.0;  
    marker.color.b = 1.0;  
    marker.color.a = 1.0;  

    // Publish the marker
    marker_pub.publish(marker);
}

/**
 * This function visualizes the mean trajectory (line) for each dynamic obstacle
 * as well as the corresponding covariance ellipses corresponding to the mean trajectory.
 * This visualization can be seen in RViz.
*/
void Rrt::visualizePrediction(ros::Publisher& line_marker_pub, 
                         ros::Publisher& ellipse_marker_pub, 
                         const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                         const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
                        ) 
{
    visualization_msgs::MarkerArray line_marker_array;
    visualization_msgs::MarkerArray ellipse_marker_array;
    int line_id = 0; // Line Marker ID counter
    int ellipse_id = 0; // Ellipse Marker ID counter
    int N_persons = all_ellipses.size();

    for (int i = 0; i < N_persons; i++){

      // Visualize trajectory
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = "world";
      line_marker.header.stamp = ros::Time::now();
      line_marker.ns = "predicted_position";
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker.id = line_id++;
      line_marker.scale.x = 0.1;  
      line_marker.color.r = 0.5;  
      line_marker.color.g = 0.4;  
      line_marker.color.b = 0.4;  
      line_marker.color.a = 1.0; 
      line_marker.pose.orientation.x = 0.0;
      line_marker.pose.orientation.y = 0.0;
      line_marker.pose.orientation.z = 0.0;
      line_marker.pose.orientation.w = 1.0;

      // Extract trajectory and ellipses for a certain dynamic obstacle (person)
      const std::vector<double>& xCoords = std::get<0>(trajectories[i]);
      const std::vector<double>& yCoords = std::get<1>(trajectories[i]);
      const std::vector<double>& zCoords = std::get<2>(trajectories[i]);
      
      const std::vector<std::tuple<double, double, Eigen::MatrixXd>>& ellipses = all_ellipses[i]; //List of tuples

      int N_trajectory_steps = xCoords.size();
      
      for (int j = 0; j < N_trajectory_steps; ++j) {
         
          // Visualize trajectory by filling a line with points
          geometry_msgs::Point point;
          point.x = xCoords[j];
          point.y = yCoords[j];
          point.z = zCoords[j];
          line_marker.points.push_back(point);

          //Extract the values to build the ellipse
          double major_length = std::get<0>(ellipses[j]);
          double minor_length = std::get<1>(ellipses[j]);
          Eigen::MatrixXd eigenvectors = std::get<2>(ellipses[j]);

          //Visualize Ellipse
          visualization_msgs::Marker ellipse_marker;
          ellipse_marker.header.frame_id = "world";
          ellipse_marker.header.stamp = ros::Time::now();
          ellipse_marker.id = ellipse_id++;
          ellipse_marker.type = visualization_msgs::Marker::SPHERE;
          ellipse_marker.pose.position.x = xCoords[j];
          ellipse_marker.pose.position.y = yCoords[j];
          ellipse_marker.pose.position.z = zCoords[j];
          ellipse_marker.color.a = 1.0;
          ellipse_marker.color.r = 0.0;
          ellipse_marker.color.g = 1.0;
          ellipse_marker.color.b = 0.0;

          // Compute orientation quaternion of covariance ellipse
          Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(eigenvectors(0), eigenvectors(1), 0.0)));
          q.x() = 0;
          q.y() = 0;
          q.normalize();
          ellipse_marker.pose.orientation.x = q.x();
          ellipse_marker.pose.orientation.y = q.y();
          ellipse_marker.pose.orientation.z = q.z();
          ellipse_marker.pose.orientation.w = q.w();

          // Give the marker the right dimensions according to the major and minor length
          ellipse_marker.scale.x = 2.0 * major_length;
          ellipse_marker.scale.y = 2.0 * minor_length;
          ellipse_marker.scale.z = 0.1;
          
          ellipse_marker_array.markers.push_back(ellipse_marker);
          
      }
      line_marker_array.markers.push_back(line_marker);    
      
      }      
    // Publish the marker arrays (all dynamic obstacles)
    line_marker_pub.publish(line_marker_array);
    ellipse_marker_pub.publish(ellipse_marker_array);
}

}  // namespace aeplanner_ns

