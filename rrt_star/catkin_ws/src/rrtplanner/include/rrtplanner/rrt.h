
#ifndef _RRT_H_
#define _RRT_H_

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <rrtplanner/rrtAction.h>
#include <actionlib/server/simple_action_server.h>

#include <kdtree/kdtree.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <cmath>

// DAEP
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rrtplanner/kalman.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <iostream>
#include <tuple>
//#include <pigain/Query.h>

#include <rrtplanner/SafePath.h>

#include "rrtplanner/Obstacle.h"
#include "rrtplanner/Obstacles.h"
#include "rrtplanner/Scenario.h"
#include "rrtplanner/BoundingBox.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <limits>
#include <std_msgs/Bool.h>

#define PI 3.14159265

namespace aeplanner_ns
{

struct RrtNode
{
  Eigen::Vector3d pos;
  RrtNode *parent{NULL};
  std::vector<RrtNode *> children;

  ~RrtNode()
  {
    for (typename std::vector<RrtNode *>::iterator node_it = children.begin();
         node_it != children.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  double cost()
  {
    if (parent)
      return (pos - parent->pos).norm() + parent->cost();
    return 0.0;
  }

  double time_cost(double drone_linear_velocity, double drone_angular_velocity)
  {
    if (this->parent)
      return this->time_to_reach(this->parent->pos, drone_linear_velocity, drone_angular_velocity) + this->parent->time_cost(drone_linear_velocity, drone_angular_velocity);
    else
      return 0.0;
  }

  double time_to_reach(const Eigen::Vector3d& pos, double drone_linear_velocity, double drone_angular_velocity) 
  {
    //Current node
    double current_x = this->pos[0];
    double current_y = this->pos[1];
    double current_z = this->pos[2];

    //New node to reach
    double target_x = pos[0];
    double target_y = pos[1];
    double target_z = pos[2];

    // Calculate Euclidean distance in x-y plane
    Eigen::Vector3d p3(this->pos[0], this->pos[1], this->pos[2]);
    Eigen::Vector3d q3(pos[0], pos[1], pos[2]);
    double euclidean_distance = (p3 - q3).norm();

    // Calculate time to move linearly to target position
    double linear_time = euclidean_distance / drone_linear_velocity;

    // Calculate total time to reach target state
    double total_time = linear_time;
    return total_time;
  }

};

  /* Floating point type */
  using FT = float;
    
  /* Utility */
  inline void clean_floating_number(FT & number) {
    if(std::abs(number) < 1e-5)
      number = 0.0;
  }

  /* SearchState: State used in search 
    ====================================*/
  class SearchState {
    public:
    Eigen::Vector3f position{};
    Eigen::Vector3f velocity{};
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
    public:
      SearchState() {}
      SearchState(FT x, FT y, FT z, FT vx = 0, FT vy = 0, FT vz = 0) {
        position << x, y, z;
        velocity << vx, vy, vz;
      }
      SearchState(Eigen::Vector3f position, Eigen::Vector3f velocity = Eigen::Vector3f(0,0,0)) {
        this->position = position;
        this->velocity = velocity;
      }

      bool operator==(const SearchState & o) {
        return position == o.position && 
               (velocity - o.velocity).norm() < 0.01;
      }
      bool operator!=(const SearchState & o) {
        return !(*this == o);
      }
      void clean_up_numbers() {
        clean_floating_number(position.x());
        clean_floating_number(position.y());
        clean_floating_number(position.z());
        clean_floating_number(velocity.x());
        clean_floating_number(velocity.y());
        clean_floating_number(velocity.z());
      }
      friend std::ostream& operator<<(std::ostream& os, const SearchState & o) {  
        os << "(x,y,z): (" << o.position.x() << "," << o.position.y() << "," << o.position.z() << ")";
        os << ", (vx,vy,vz): (" << o.velocity.x() << "," << o.velocity.y() << "," << o.velocity.z() << ")";  
        return os;
      }

  };


class Rrt
{
  public:
    Rrt(const ros::NodeHandle &nh);
    void octomapCallback(const octomap_msgs::Octomap &msg);
  
    void execute(const rrtplanner::rrtGoalConstPtr &goal);
    void visualizeGoals(std::vector<geometry_msgs::Pose> goals);
    void visualizeNode(geometry_msgs::Point pos, int id = 0);
    void visualizePose(geometry_msgs::Pose pose, int id = 0);
    void visualizeEdge(RrtNode *node, int id = 0);
    void visualizePath(RrtNode *node);

    Eigen::Vector3d sample();
    RrtNode *chooseParent(kdtree *kd_tree, Eigen::Vector3d, double l);
    void rewire(kdtree *kd_tree, RrtNode *new_node, double l, double r, double r_os);
    Eigen::Vector3d getNewPosNormalized(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
    bool collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r);
    RrtNode *addNodeToTree(kdtree *kd_tree, RrtNode *parent, Eigen::Vector3d new_pos);
    RrtNode *getGoal(kdtree *goal_tree, RrtNode *new_node, double l, double r, double r_os);
    std::pair<nav_msgs::Path, std::vector<double>> getBestPath(std::vector<RrtNode*> goals);
    std::vector<geometry_msgs::Pose> checkIfGoalReached(kdtree *goal_tree, RrtNode *new_node, double l, double r, double r_os);

    // SDMP
    void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg); //Currently not used
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void obstacleCallback(const rrtplanner::Obstacles::ConstPtr& msg);    //Currently not used
    void scenarioCallback(const rrtplanner::Scenario::ConstPtr& msg);
    void setGoalState(SearchState goal_state);
    void planning_cycle(int min_nodes);
    bool new_nav_goal_ = false;
    SearchState dji_state_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    SearchState goal_state_ {std::numeric_limits<float>::infinity(), 
                             std::numeric_limits<float>::infinity(), 
                             std::numeric_limits<float>::infinity(), 
                             0.0, 
                             0.0, 
                             0.0};
    
    nav_msgs::Path plan_;
    std::vector<double> time_steps_;
    bool has_valid_pose_ = false;
    bool has_valid_vel_ = false;
    bool has_valid_goal_ = false;
    bool has_valid_bounds_ = false;
    bool has_valid_state() { return has_valid_pose_ && 
                                    has_valid_vel_  && 
                                    has_valid_goal_ && 
                                    new_nav_goal_   &&
                                    has_valid_bounds_;
                            }
    double location_tolerance_;
    double dji_safety_radius_;
    ros::Publisher& getPosePub() { return pose_pub_; }
    ros::Publisher& getGoalReachedPub() { return reached_goal_pub_; }
    ros::Publisher& getTrajectoryPub() { return trajectory_pub_; }
    std::map<std::string, KalmanFilter> kalman_filters;
    bool safePath(nav_msgs::Path path, std::vector<double> time_steps);
    double getDroneLinearVelocity() { return drone_linear_velocity_; }
    std_msgs::Bool reached_goal_msg;


    // Data parameters
    ros::Duration totalTime_{};
    double costToGoal_{};
    double timeToReach_{};
    int planSize_{};
    int evalStates_{};
    bool foundPlan_{};

    // Visualization
    void visualizeGroundTruth(ros::Publisher& marker_pub, const int& object_id, const geometry_msgs::Pose& pose, float radius);
    void visualizePrediction(ros::Publisher& line_marker_pub, 
                         ros::Publisher& ellipse_marker_pub, 
                         const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                         const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
                        );


 

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<octomap::OcTree> ot_;

    ros::Subscriber octomap_sub_;
    actionlib::SimpleActionServer<rrtplanner::rrtAction> as_;

    std::string frame_id_;

    ros::Publisher path_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher ground_truth_dynamic_objects_pub_;
    ros::Publisher pred_marker_pub_;
    ros::Publisher covariance_marker_pub_;
    ros::Publisher reached_goal_pub_;


    double min_nodes_;
    double bounding_radius_;
    double bounding_overshoot_;
    double extension_range_;
    std::vector<double> boundary_min_;
    std::vector<double> boundary_max_;
    int KFiterations_;
    double dt_;

    double drone_linear_velocity_;
    double drone_angular_velocity_;

    //SAFE PATH with DAEP
    std::map<std::string, std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> dynamic_objects{};
    std::map<std::string, rrtplanner::Obstacle> static_objects{};

    ros::Subscriber human_sub_;
    void updateHumanPositions(const gazebo_msgs::ModelStates& model_states);
    std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories();
    bool dynamic_mode_;
    bool safePathSrvCallback(rrtplanner::SafePath::Request& request, rrtplanner::SafePath::Response& response);
    ros::ServiceServer safe_path_srv_;
    bool isCollision(const geometry_msgs::PoseStamped& posestamped_parent, const geometry_msgs::PoseStamped& posestamped, double time_of_arrival,
                          std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data);
    bool checkCollision(double t, 
                        Eigen::Vector3d point, 
                        std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                        std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses,
                        Eigen::Vector3d parent);
                          
    bool lineIntersectsCircle(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                            const Eigen::Vector3d center, double radius);

    int getCovarianceIndex(double max_time_step, double time_step, double t);
    std::vector<std::tuple<double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);
    bool checkIntersectStaticObstacle(Eigen::Vector3d p1, Eigen::Vector3d p2, double bounding_radius);
    bool lineInsideBox(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector3d box_center, Eigen::Vector3d box_size);
    std::vector<Eigen::Vector3d> lineIntersectBox(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector3d box_center, Eigen::Vector3d box_size);
    bool between(Eigen::Vector3d x, Eigen::Vector3d axis, Eigen::Vector3d p1, Eigen::Vector3d p2);
    bool pointTooCloseToWall(Eigen::Vector3d point, Eigen::Vector3d box_center, Eigen::Vector3d box_size);


    // SDMP
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber scenario_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber goal_sub_;

  
};
  

float CylTest_CapsFirst(const octomap::point3d &pt1,
                        const octomap::point3d &pt2,
                        float lsq, float rsq, const octomap::point3d &pt);
} // namespace aeplanner_ns

#endif
