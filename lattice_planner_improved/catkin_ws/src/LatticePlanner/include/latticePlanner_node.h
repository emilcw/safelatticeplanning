#pragma once
#ifndef __LATTICE_PLANNER_NODE_H__
#define __LATTICE_PLANNER_NODE_H__

// std
#include <random>


// C-library
#include <signal.h>

// ROS - msgs
#include "ros/ros.h"
#include <std_msgs/Bool.h>
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
#include <nav_msgs/OccupancyGrid.h>
#include <mav_msgs/common.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/BoundingBoxQueryResponse.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

// headers
#include "LatticePlanner.h"
#include "Scenario.h"
#include "Logger.h"
#include "lattice_planner/plantime.h"
#include "lattice_planner/Obstacle.h"
#include "lattice_planner/Obstacles.h"
#include "lattice_planner/Scenario.h"
#include "lattice_planner/BoundingBox.h"


using namespace std;
using namespace octomap;

#define LOG(tag,time,value) logger.log(tag,time,value)


class LatticePlannerNode {

public:
    bool new_nav_goal_;
    bool use_predictions_ = true;
    bool adaptive_planning_ = false;
    bool survival_planning_ = false;
    bool emergency_trajectories_ = false;

    bool previous_iteration_was_emergency = false;
    TrajectoryState anchor_point;

    float location_tolerance_{};
    int seed{};
    std_msgs::Bool reached_goal_msg;
    ros::Publisher reached_goal_pub_;

    LatticePlanner lattice_planner_;
    LatticePlanner object_lattice_planner_; //This is the Lattice Planner that the dynamic obstacles use. The DJI100 does not use this one!
    Profiler object_profiler;
    Logger logger;
    SearchState dji_state_;
    SearchState last_dji_state_ {std::numeric_limits<float>::infinity(), 
                             std::numeric_limits<float>::infinity(), 
                             std::numeric_limits<float>::infinity(), 
                             0.0, 
                             0.0, 
                             0.0};

    SearchState goal_state_ {std::numeric_limits<float>::infinity(), 
                             std::numeric_limits<float>::infinity(), 
                             std::numeric_limits<float>::infinity(), 
                             0.0, 
                             0.0, 
                             0.0};
    
    TrajectoryState collision_state_ {std::numeric_limits<float>::infinity(), 
                                      std::numeric_limits<float>::infinity(), 
                                      std::numeric_limits<float>::infinity(), 
                                      0.0, 
                                      0.0, 
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0};
    double time_until_collision_ {std::numeric_limits<double>::infinity()};
    

private:
    bool has_valid_pose_;
    bool has_valid_vel_;
    bool has_valid_goal_;
    bool has_valid_scenario_;
    bool has_valid_obstacles_;
    bool first_replanning = true;
    bool last_state_receeding_horizon;
    bool last_state_emergency_avoidance;
    
    double grid_size_;
    int free_cell_index{0};
    int occupied_cell_index{0};
    int num_static_obstacles = 0;

    vector<State> current_plan_;
    vector<TrajectoryState> current_trajectory_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Time previous_cycle_time_;
    trajectory_msgs::MultiDOFJointTrajectory Dji_reference_trajectory_;
    visualization_msgs::Marker line_list_;

    octomap::OcTree* octree;

    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber scenario_sub_;
    ros::Subscriber oct_sub_;
    
    ros::Publisher trajectory_pub_;
    ros::Publisher trajectory_id_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher teleport_pub_;

    // Visualization
    ros::Publisher plan_trajectory_pub_;
    ros::Publisher plan_trajectory_safety_radius_pub_;
    ros::Publisher dji_hitbox_pub_;
    ros::Publisher plan_obstacles_pub_;
    ros::Publisher obstacle_plan_trajectory_pub_;
    ros::Publisher travelled_path_pub_;
    ros::Publisher closed_set_pub_;
    ros::Publisher octomap_pub_;
    ros::Publisher octomap_plane_occupied_grid_pub_;
    ros::Publisher anchor_point_pub_;
    ros::Publisher end_point_pub_;
    ros::Publisher debug_pose_pub_;

    Profiler profiler;
    lattice_planner::plantime Dji_reference_plan_;

public:

  // Constructors
  LatticePlannerNode() {}
  ~LatticePlannerNode() {}

  // General
  bool initialize(ros::NodeHandle & nh, ros::NodeHandle & private_nh);
	std::pair<bool, bool> planning_cycle(double planning_time, double replanning_time, int iteration, bool visualize_obstacles = false);    

  // Callbacks
  void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void obstacleCallback(const lattice_planner::Obstacles::ConstPtr& msg);
  void scenarioCallback(const lattice_planner::Scenario::ConstPtr& msg);
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);


  // Visualization
  void publish_visualization_obstacles() { std::vector<Obstacle> o = lattice_planner_.get_obstacles(); publish_visualization_obstacles(o); }
  void publish_visualization_closed_set();
  void publish_visualization_octomap(double width, double height, double stepsize);
  void publish_visualization_octomap_plane_grid();
  void publish_visualization_obstacle_plan_trajectory(std::vector<Obstacle> obstacles);

  // DJI
  bool has_valid_state() { return has_valid_pose_ && 
                                has_valid_vel_  && 
                                has_valid_goal_ && 
                                has_valid_scenario_ && 
                                has_valid_obstacles_; }


  vector<State>& get_current_plan() { return current_plan_; };
  vector<TrajectoryState>& get_current_trajectory() { return current_trajectory_; };

  // Obstacles
  void add_obstacle(Obstacle o, bool dynamic);
  void clear_obstacles() { lattice_planner_.clear_obstacles(); }
  void set_valid_obstacles(bool const& has_valid_obstacles) { has_valid_obstacles_ = has_valid_obstacles; }
  bool has_valid_obstacles() { return has_valid_obstacles_; }
  void update_obstacles(double time);
  void observe_obstacles(double time);
  std::vector<Obstacle> get_all_obstacles() {return lattice_planner_.get_obstacles(); }

  // Volume
  void set_allowed_volume(AABB allowed_volume) { lattice_planner_.set_allowed_volume(allowed_volume); }
  void set_allowed_volume(double minX, double maxX,
                          double minY, double maxY,
                          double minZ, double maxZ) 
  {
    lattice_planner_.set_allowed_volume(minX, maxX, minY, maxY, minZ, maxZ);
  }

  // Collisions
 std::tuple<bool, double, TrajectoryState> plan_in_collision(std::vector<TrajectoryState> current_trajectory, double duration);
 std::tuple<bool, double, TrajectoryState> plan_in_collision(std::vector<TrajectoryState> current_trajectory, double duration, int secondary_traj_start_index);

  // Costs
  double calculate_plan_cost(std::vector<State> & previous_plan, double duration);

  // Miscellaneous 
  void setGoalState(SearchState goal_state);
  SearchState round_search_state(SearchState state);
  SearchState round_search_state_and_collision_check(SearchState state) ;
  SearchState draw_random_free_position(AABB limit, bool static_only, std::default_random_engine& rng);
  Profiler& get_profiler() { return profiler; };
  visualization_msgs::Marker get_travelled_path() { return line_list_; };
  double travelled_distance_from_trajectory(visualization_msgs::Marker line_list);

  bool is_state_initalized(SearchState state)
  {
    return state.position.x() != std::numeric_limits<float>::infinity() && 
           state.position.y() != std::numeric_limits<float>::infinity() &&
           state.position.z() != std::numeric_limits<float>::infinity();
  }

private: //Member functions

    void generate_global_trajectory();
    void generate_global_plan();
    bool is_new_plan(std::vector<State> new_plan, std::vector<State> prev_plan, int start_index);
    void stop_command();
    void time_adjust_plan(std::vector<State> & plan, std::vector<TrajectoryState> & trajectory, double dt);

    int next_path_state(double time_duration, std::vector<State> & plan);
    std::vector<TrajectoryState> calculate_trajectory_prefix(double duration,
                                                             State & startState,
                                                             std::vector<TrajectoryState> & trajectory);

    std::vector<TrajectoryState> calculate_trajectory_prefix2(double duration, TrajectoryState &startState, std::vector<TrajectoryState> &trajectory);

    
    int next_trajectory_state(SearchState & current_state, const std::vector<TrajectoryState> & trajectory,
                              int min_index, int max_index);

    int next_trajectory_state(double time_duration, std::vector<TrajectoryState> &trajectory);
    
    int find_closest_trajectory_state(SearchState & state, const std::vector<TrajectoryState> & trajectory,
                                      int min_index, int max_index);

    void print_trajectory(std::vector<TrajectoryState> & trajectory) 
    {
      std::stringstream ss;
      int n = 0;
      for(std::vector<TrajectoryState>::iterator state = trajectory.begin(); state != trajectory.end(); state++) {
       ss << n << "] " << *state << "\n";
        n++;
      }
      ROS_ERROR_STREAM("print_trajectory(...):\n" << ss.str());
    }

    void print_plan(std::vector<State> & plan) 
    {
      std::stringstream ss;
      int n = 0;
      for(std::vector<State>::iterator state = plan.begin(); state != plan.end(); state++) {
       ss << n << "] " << *state << "\n";
        n++;
      }
      ROS_ERROR_STREAM("print_plan(...):\n" << ss.str());
    }

    int calculate_secondary_plan_index(std::vector<State> & current_plan);
    int calculate_secondary_trajectory_start_index(std::vector<TrajectoryState> & current_trajectory,
                                                                                std::vector<State> & current_plan,
                                                                                int secondary_plan_start_index);

    // Visualization
    void publish_visualization_plan_trajectory(trajectory_msgs::MultiDOFJointTrajectory & trajectory, int secondary_traj_index_start);
    void publish_visualization_plan_trajectory_safety_radius(trajectory_msgs::MultiDOFJointTrajectory & trajectory, int secondary_traj_index_start);
    void publish_visualization_obstacles(std::vector<Obstacle> obstacles);
    void publish_visualization_obstacles_hitbox(std::vector<Obstacle> obstacles);
    void publish_visualization_obstacles_predictions(std::vector<Obstacle> obstacles, bool conservative_predictions);
    void publish_visualization_travelled_path();
    void publish_visualization_goal(SearchState & state);
    void publish_dji_hitbox();
    void publish_anchor_point(SearchState state);
    void publish_off_lattice_pos(SearchState state);
    void debug_pose(SearchState state);

};

// --- Auxiliary helper functions ---
void shutdown_gracefully();
void mySigintHandler(int sig) { shutdown_gracefully(); }
int find_closest_path_state(SearchState state, const std::vector<State*> & path);
double projected_distance(State * s1, State * s2, SearchState & p);
double projected_distance(SearchState & s1, SearchState & s2, SearchState & p);
double roundPartial(double value, double resolution) { return round(value / resolution) * resolution; }
SearchState draw_random_position(AABB limit);

/* ---------------------- [DEPRECATED] ----------------------
void apply_scenario(LatticePlannerNode * lattice_planner_node, 
                    Scenario & scenario);
void run_test(LatticePlannerNode * lattice_planner_nde, 
              std::string test, 
              std::string scenario, 
              double planning_time, 
              double replanning_time);
              
//int countCollisions(LatticePlannerNode *lattice_planner_node); [DEPRECATED]
//void log_obstacle_collisions(double time, LatticePlannerNode * lattice_planner_node); [ÐEPRECATED]
void add_advanced_obstacle(double x, [DEPRECATED, used in SetupScenario]
                             double y, 
                             double z, 
                             double maxSpeed, 
                             std::string type, 
                             AABB limit);
//void initialize_obstacles(); [DEPRECATED, used in SetupScenario]
// Scenario or other test modes
//void evaluate_primitive_execution(); [DEPRECATED]
//TestResult run_performance_test(Scenario & scenario, [DEPRECATED]
//                                double planning_time, 
//                                double replanning_time);
//void fly_to_blocking(geometry_msgs::Point position); [DEPRECATED]
//void teleport(geometry_msgs::Point position); [DEPRECATED] 

*/

#endif //__LATTICE_PLANNER_NODE_H__
