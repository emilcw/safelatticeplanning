#pragma once
#ifndef __LATTICE_PLANNER_NODE_H__
#define __LATTICE_PLANNER_NODE_H__

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include <mav_msgs/common.h>
#include <boost/thread.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <math.h>
#include <ctime>
#include <signal.h>
#include <random>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/BoundingBoxQueryResponse.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <std_msgs/Bool.h>

#include "LatticePlanner.h"
#include "Scenario.h"
#include "Logger.h"

#include "setup_scenario/plantime.h"
#include "setup_scenario/Obstacle.h"
#include "setup_scenario/Obstacles.h"
#include "setup_scenario/Scenario.h"
//#include "setup_scenario/Hitbox.h"


using namespace std;
using namespace octomap;

#define LOG(tag,time,value) logger.log(tag,time,value)
//#define Log(tag,time,value)

class LatticePlannerNode {

public:

  LatticePlannerNode() {}
  ~LatticePlannerNode() {}

  bool initialize(ros::NodeHandle & nh, ros::NodeHandle & private_nh);

	bool planning_cycle(double planning_time, double replanning_time);

  void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void reachedGoalPlannerCallback(const std_msgs::Bool::ConstPtr& msg);
  void velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void obstCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void obstCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
  void update_obstacle(const geometry_msgs::Pose p);

  void evaluate_primitive_execution();

  void setGoalState(SearchState goal_state);

  void goal_reached() { lattice_planner_.goal_reached(); }

  bool has_valid_state() { return has_valid_pose_ && has_valid_vel_ && has_valid_goal_; }

  void publish_visualization_obstacles() { std::vector<Obstacle> o = lattice_planner_.get_obstacles(); publish_visualization_obstacles(o); }
  void add_obstacle(Obstacle o, bool dynamic);
  void clear_obstacles() { lattice_planner_.clear_obstacles(); }
  std::vector<Obstacle> get_all_obstacles() {return lattice_planner_.get_obstacles(); }

  SearchState round_search_state(SearchState state);

  void initialize_obstacles();
  void update_obstacles(double time);
  void observe_obstacles(double time);
  void publish_scenario(Scenario & scenario);
  void publish_limits(Scenario & scenario);
  void publish_obstacles_info();

  void set_allowed_volume(AABB allowed_volume) { lattice_planner_.set_allowed_volume(allowed_volume); }
  void set_allowed_volume(double minX, double maxX,
                        double minY, double maxY,
                        double minZ, double maxZ) {
    lattice_planner_.set_allowed_volume(minX, maxX, minY, maxY, minZ, maxZ);
  }

  bool plan_in_collision(std::vector<TrajectoryState> current_trajectory, double duration);
  bool plan_in_collision(std::vector<TrajectoryState> current_trajectory, double duration, int secondary_traj_start_index);
  double calculate_plan_cost(std::vector<State> & previous_plan, double duration);

  // For development use only
  void fly_to_blocking(geometry_msgs::Point position);
  void teleport(geometry_msgs::Point position);
  void publish_visualization_closed_set();
  void publish_visualization_octomap(double width, double height, double stepsize);
  void publish_visualization_octomap_plane_grid();

  // For scenarios
  void add_advanced_obstacle(double x, double y, double z, double maxSpeed, std::string type, AABB limit);

  SearchState draw_random_free_position(AABB limit, bool static_only, std::default_random_engine& rng);


  TestResult run_performance_test(Scenario & scenario, double planning_time, double replanning_time);

private:

    void generate_global_trajectory();
    void generate_global_plan();

    void time_adjust_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory, double dt);

    //int next_path_state(SearchState & state, std::vector<State> & plan);
    int next_path_state(double time_duration, std::vector<State> & plan);
    std::vector<TrajectoryState> calculate_trajectory_prefix(double duration,
                                                             State & startState,
                                                             std::vector<TrajectoryState> & trajectory);

    void publish_plan(std::vector<State> plan);


    void publish_visualization_plan_trajectory(trajectory_msgs::MultiDOFJointTrajectory & trajectory,
                                               int secondary_traj_index_start);
    void publish_visualization_plan_trajectory_safety_radius(trajectory_msgs::MultiDOFJointTrajectory & trajectory,
                                                             int secondary_traj_index_start);
    void publish_visualization_obstacles(std::vector<Obstacle> obstacles);
    void publish_visualization_obstacles_hitbox(std::vector<Obstacle> obstacles);
    void publish_visualization_obstacles_predictions(std::vector<Obstacle> obstacles, bool conservative_predictions);
    void publish_visualization_obstacle_plan_trajectory(std::vector<Obstacle> obstacles);

    void publish_visualization_travelled_path();
    void publish_visualization_goal(SearchState & state);
    void publish_visualization_vehicle();
    
    int next_trajectory_state(SearchState & current_state, const std::vector<TrajectoryState> & trajectory,
                              int min_index, int max_index);
    int find_closest_trajectory_state(SearchState & state, const std::vector<TrajectoryState> & trajectory,
                                      int min_index, int max_index);

    // Debug
    void print_trajectory(std::vector<TrajectoryState> & trajectory) {
      std::stringstream ss;
      int n = 0;
      for(std::vector<TrajectoryState>::iterator state = trajectory.begin(); state != trajectory.end(); state++) {
       ss << n << "] " << *state << "\n";
        n++;
      }
      ROS_ERROR_STREAM("print_trajectory(...):\n" << ss.str());
    }
    void print_plan(std::vector<State> & plan) {
      std::stringstream ss;
      int n = 0;
      for(std::vector<State>::iterator state = plan.begin(); state != plan.end(); state++) {
       ss << n << "] " << *state << "\n";
        n++;
      }
      ROS_ERROR_STREAM("print_plan(...):\n" << ss.str());
    }

    bool is_new_plan(std::vector<State> new_plan, std::vector<State> prev_plan, int start_index);

    void stop_command();

    int calculate_secondary_plan_index(std::vector<State> & current_plan);
    int calculate_secondary_trajectory_start_index(std::vector<TrajectoryState> & current_trajectory,
                                                                                std::vector<State> & current_plan,
                                                                                int secondary_plan_start_index);



public:
    SearchState dji_state_;
    SearchState goal_state_ {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0.0, 0.0, 0.0};
    bool new_nav_goal_;
    Logger logger;

    LatticePlanner lattice_planner_;

    int seed;
    bool use_predictions = true;
    float location_tolerance_{};
    visualization_msgs::Marker line_list_;
    std_msgs::Bool reached_goal_msg;
    bool reached_goal_from_planner_;
    ros::Publisher reached_goal_pub_, nav_goal_pub_, obstacles_pub_, scenario_pub_, limits_pub_;
    ros::Publisher collision_pub_, clock_start_pub_;

private:

    octomap::OcTree* octree;

    double grid_size_;

    int num_static_obstacles = 0;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;


    bool has_valid_pose_;
    bool has_valid_vel_;
    bool has_valid_goal_;

    bool first_replanning = true;
    bool last_state_receeding_horizon;
    bool last_state_emergency_avoidance;
    int free_cell_index{0};
    int occupied_cell_index{0};


    ros::Time previous_cycle_time_;

    Profiler profiler;

    trajectory_msgs::MultiDOFJointTrajectory Dji_reference_trajectory_;
    setup_scenario::plantime Dji_reference_plan_;

    vector<State> current_plan_;
    vector<TrajectoryState> current_trajectory_;

    ros::Subscriber pose_sub_, vel_sub_, goal_sub_, obst_sub_, obst_sub2_,oct_sub_, reached_goal_planner_;
    ros::Publisher trajectory_pub_, pose_pub_, teleport_pub_, plan_pub_,trajectory_id_pub_;

    // Visualization and debugging
    ros::Publisher plan_trajectory_pub_;
    ros::Publisher plan_trajectory_safety_radius_pub_;
    ros::Publisher plan_vehicle_pub_;
    ros::Publisher octomap_pub_;
    ros::Publisher octomap_plane_occupied_grid_pub_;
    ros::Publisher plan_obstacles_pub_;
    ros::Publisher obstacle_plan_trajectory_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher closed_set_pub_;
    ros::Publisher travelled_path_pub_;

public:
    // For scenario
    LatticePlanner object_lattice_planner_; //This is the lattice planner that the dynamic obstaclse use. The DJI does not use this one!
    Profiler object_profiler;
};



int find_closest_path_state(SearchState state, const std::vector<State*> & path);

double projected_distance(State * s1, State * s2, SearchState & p);
double projected_distance(SearchState & s1, SearchState & s2, SearchState & p);

/*
 * Lattice planning scenarios.
 * Each scenario includes a configuration of
 *  - starting location
 *  - goal location
 *  - AABB boundary of search region
 *  - list of static and dynamic obstacles (with necessary internal states)
 * The lattice planner is setup to be ready to execute the scenario.
 * If the physical simulator is running then the UAV will be moved using control commands
 * to the starting location before the scenario starts.
 */
void apply_scenario(LatticePlannerNode * lattice_planner_node, Scenario & scenario);

void run_test(LatticePlannerNode * lattice_planner_nde, std::string test, std::string scenario, double planning_time, double replanning_time);

void evaluate_primitive_execution(LatticePlannerNode * lattice_planner_node);

geometry_msgs::Point create_point(double x, double y, double z);
SearchState draw_random_position(AABB limit);

void log_obstacle_collisions(double time, LatticePlannerNode * lattice_planner_node);



#endif //__LATTICE_PLANNER_NODE_H__
