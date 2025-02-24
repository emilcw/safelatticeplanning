#include <rrtplanner/rrt.h>
#include <tf/transform_datatypes.h>

/*
This function should given a plan, genearte a feasible trajectory for the NMPC to follow.
*/
trajectory_msgs::MultiDOFJointTrajectory generate_trajectory_from_plan(nav_msgs::Path plan)
{

  trajectory_msgs::MultiDOFJointTrajectory Dji_reference_trajectory_;
  geometry_msgs::Transform transform;
  geometry_msgs::Twist velocities;
  geometry_msgs::Twist accelerations;

  for (int n = 0; n < plan.poses.size(); n++) {

    trajectory_msgs::MultiDOFJointTrajectoryPoint ref_point;

    transform.translation.x = plan.poses[n].pose.position.x;
    transform.translation.y = plan.poses[n].pose.position.y;
    transform.translation.z = plan.poses[n].pose.position.z;

    velocities.linear.x = 0.1; 
    velocities.linear.y = 0.1; 
    velocities.linear.z = 0.1;

    geometry_msgs::Quaternion quat = plan.poses[n].pose.orientation;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double r = roll;  
    double p = pitch; 
    double y = yaw;

    double liner_drag_coefficient = 0.01;

    double useRefCtrl = 1;
    accelerations.linear.x = useRefCtrl * (cos(y) * sin(p) * cos(r) + sin(y) * sin(r)) * 1 - liner_drag_coefficient * 0.1;
    accelerations.linear.y = useRefCtrl * (sin(y) * sin(p) * cos(r) - cos(y) * sin(r)) * 1 - liner_drag_coefficient * 0.1;
    accelerations.linear.z = useRefCtrl * (cos(p) * cos(r) * 1 - 9.806);

    transform.rotation = plan.poses[n].pose.orientation;

    ref_point.transforms.push_back(transform);
    ref_point.velocities.push_back(velocities);
    ref_point.accelerations.push_back(accelerations);

    //ref_point.time_from_start = ros::Duration(this->current_trajectory_[n].time);

    Dji_reference_trajectory_.points.push_back(ref_point);
  }

  return Dji_reference_trajectory_;
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt");
  ROS_INFO("Global RRT-planner initialized");
  ros::NodeHandle nh;

  aeplanner_ns::Rrt rrt(nh);

  ros::Rate loop_rate(10);

  /*-------------------WRITE DATA TO FILE -------------------*/
  
  // Open file streams to write simulation data
  std::ofstream logfile, summaryfile;
  std::string homePath = std::getenv("HOME");
  logfile.open(homePath + "/data/logfile.csv");
  summaryfile.open(homePath + "/data/summary.csv");

  // Insert headers
  logfile << "Iteration, "
          << "Simulation Time [s], " 
          << "Total Search Time [s], "
          << "Cost to Goal [m], "
          << "Time to reach Goal [s], "
          << "Plan Size [#State], "
          << "Evaluated States [#State], "
          << "Dynamic Obstacle Collisions [#], "
          << "Future plan safe? [bool], "
          << "Flying? [bool], "
          << "Safe Path Time [s], "
          << "Found plan? [bool], "<< std::endl;


  summaryfile << "Total Simulation Time [s], "
              << "Accumulated Total Search Time [s], "
              << "Accumulated Cost to Goal [m], "
              << "Accumulated Plan Size [#State], "
              << "Accumulated Evaluated States [#State], "
              << "Accumulated Travelled Distance [m], "
              << "Accumulated Safe Path Time [s], "
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
          << 0 << std::endl;

  // Data variables
  bool firstTime{true};
  ros::Time start;
  ros::Duration elapsed;
  ros::Duration safePathTime;
  long iteration{};
  ros::Duration accumulatedTotalTime{};
  ros::Duration accumulatedSafePathTime{};
  long min_nodes = 200; //Initial search amount
  long search_counter{};

  double accumulatedCost{};
  double accumulatedTravelledDistance{};
  int accumulatedPlanSize{};
  int accumulatedEvaluatedStates{};
  int dynamicObstacleCounter{};

  nav_msgs::Path travelled_path{};
  geometry_msgs::PoseStamped start_pose;
  start_pose.pose.position.x = rrt.dji_state_.position.x();
  start_pose.pose.position.y = rrt.dji_state_.position.y();
  start_pose.pose.position.z = rrt.dji_state_.position.z();
  travelled_path.poses.push_back(start_pose);

  rrt.reached_goal_msg.data = false;

  while(ros::ok())
  {
    
    bool planSafe{true};
    bool flying{false};

    // Reached goal?
    double goal_dist = (rrt.dji_state_.position - rrt.goal_state_.position).norm();
    ROS_INFO_STREAM_THROTTLE(1, "goal_dist: " << goal_dist << " current_state: " << rrt.dji_state_ << " goal_state: " << rrt.goal_state_);

    accumulatedTravelledDistance = getPathLength(travelled_path);

    summaryfile << elapsed << ", "
                << accumulatedTotalTime << ", " 
                << accumulatedCost << ", "
                << accumulatedPlanSize << ", "   
                << accumulatedEvaluatedStates << ", " 
                << accumulatedTravelledDistance << ", "
                << accumulatedSafePathTime << ", "
                << dynamicObstacleCounter << std::endl; 

    if (goal_dist < rrt.location_tolerance_)
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
        rrt.reached_goal_msg.data = true;
        rrt.getGoalReachedPub().publish(rrt.reached_goal_msg);
      }
      break;
    }

    // Planning cycle
    if(rrt.has_valid_state())
    {
      ROS_INFO_STREAM("Temporal RRT* -- Planning Iteration: " << iteration << " -- ");


      if(firstTime)
      {
        start = ros::Time::now();
        firstTime = false;
      }

      rrt.planning_cycle(min_nodes);

      elapsed = ros::Time::now() - start;
      ros::Duration totalTime = rrt.totalTime_;
      double costToGoal = rrt.costToGoal_;
      double timeToReach = rrt.timeToReach_;
      int planSize = rrt.planSize_;
      int evalStates = rrt.evalStates_;
      int currentCollisions{};
      safePathTime = ros::Duration(0.0);
      bool foundPlan = rrt.foundPlan_;
      search_counter++;

      if(!foundPlan && search_counter % 10)
      {
        min_nodes += 100;
      }

      if(foundPlan)
      {
        search_counter = 0;
        min_nodes = 200;
      }

      accumulatedTotalTime += totalTime;
      accumulatedCost += costToGoal;
      accumulatedPlanSize += planSize;
      accumulatedEvaluatedStates += evalStates;
      dynamicObstacleCounter += currentCollisions;

      logfile << ++iteration << ", "  //OK
              << elapsed << ", "      //OK
              << totalTime << ", "    //OK
              << costToGoal << ", "   //OK
              << timeToReach << ", "  //OK, but rough since we assume velocity = 1 m/s at all times
              << planSize << ", "     //OK
              << evalStates << ", "   //OK
              << currentCollisions << ", " //NOT USED
              << planSafe << ", "     // OK
              << flying << ", "                       //OK
              << safePathTime << ", "                 //OK
              << foundPlan << std::endl; //OK

      if(!rrt.plan_.poses.empty())
      {
        ROS_INFO_STREAM("Temporal RRT* - FOUND PLAN!");

        rrt.new_nav_goal_ = false;

        //Alt 1: Going to end position directly
        /*
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose = rrt.plan_.poses.back().pose;
        msg.pose.orientation.w = 1.0;
        rrt.getPosePub().publish(msg);
        */
        
        //Alt 2: Visit each pose in plan with /command/pose - Works quite well
        int look_ahead_horizon = 5;
        double total_time = 0.0;
        nav_msgs::Path path = rrt.plan_;
        std::vector<double> time_steps = rrt.time_steps_;

        for (int i = 0; i < rrt.plan_.poses.size(); i++)
        {
          ROS_INFO_STREAM("RRT* -- Flying Iteration: " << i << " -- ");

          
          /*

          ###

          NOT USED IN REGULAR RRT

          ###

          if(path.poses.size() < look_ahead_horizon)
          {
            look_ahead_horizon = path.poses.size();
          }

          //If we are near the end, make sure the subpath and sub-time-steps does
          //not go out of bounds, instead we should look at the remaining poses
          if( (path.poses.size() - i) < look_ahead_horizon)
          {
            look_ahead_horizon--;
          }
          
          //Extract correct future subpath
          nav_msgs::Path subpath;
          std::vector<geometry_msgs::PoseStamped> subposes(path.poses.begin() + i, path.poses.begin() + i + look_ahead_horizon);
          subpath.poses = subposes;

          //Extract correct future time stamps
          std::vector<double> sub_time_steps(time_steps.begin() + i, time_steps.begin() + i + look_ahead_horizon);
          
          ros::Time start_safe_path = ros::Time::now();
          planSafe = rrt.safePath(subpath, sub_time_steps);
          safePathTime = ros::Time::now() - start_safe_path;

          // Count down time_steps when approaching goal
          double time_to_remove = time_steps[i];
          for(auto& element : time_steps)
          {
            if(i <= time_steps.size() - 1)
            {
              element = element - time_to_remove;
            }
          }
          */  

          // Data collection during flight
          elapsed = ros::Time::now() - start;
          totalTime = ros::Duration(0.0);
          evalStates = 0;
          flying = true;
          foundPlan = false; 
          
          nav_msgs::Path remaining_path;
          std::vector<geometry_msgs::PoseStamped> remaining_poses(path.poses.begin() + i, path.poses.end());
          remaining_path.poses = remaining_poses;
          double distance_left = getPathLength(remaining_path);
          timeToReach = 0.0; //time_steps.back();
          accumulatedSafePathTime += safePathTime;

          logfile << ++iteration << ", "                  //OK
                  << elapsed << ", "                      //OK
                  << totalTime << ", "                    //OK, no planning during flight
                  << distance_left << ", "                //OK, should decrease
                  << timeToReach << ", "                  //OK, but rough since we assume velocity = 1 m/s at all times
                  << remaining_path.poses.size() << ", "  //OK
                  << evalStates << ", "                   //OK, no evaluation during flight
                  << currentCollisions << ", "            //NOT USED
                  << planSafe << ", "                     // Is future path safe?
                  << flying << ", "                       // OK
                  << safePathTime << ", "                 //OK
                  << foundPlan << std::endl;              //OK


          summaryfile << elapsed << ", "
                << accumulatedTotalTime << ", " 
                << accumulatedCost << ", "
                << accumulatedPlanSize << ", "   
                << accumulatedEvaluatedStates << ", " 
                << accumulatedTravelledDistance << ", "
                << accumulatedSafePathTime << ", "
                << dynamicObstacleCounter << std::endl; 

          if(!planSafe){
              ROS_WARN("SAFE_PATH: Future path not safe, replanning...");
              rrt.new_nav_goal_ = true;
              break;
          }

          geometry_msgs::PoseStamped msg;
          msg.header.stamp = ros::Time::now();
          msg.pose = rrt.plan_.poses[i].pose;
          msg.pose.orientation.w = 1.0;
          
          ros::Time start_fly;
          ros::Duration elapsed_fly;
          start_fly = ros::Time::now();

          rrt.getPosePub().publish(msg);

          Eigen::Vector3f target (rrt.plan_.poses[i].pose.position.x, 
                                  rrt.plan_.poses[i].pose.position.y, 
                                  rrt.plan_.poses[i].pose.position.z);

          double dist = std::numeric_limits<double>::infinity();
          
          while(dist > 0.15)
          {
            dist = (rrt.dji_state_.position - target).norm(); 
            //Needed to update dji_state_
            ros::spinOnce();
            loop_rate.sleep();
          }

          //Add the pose as visited if we reached it
          travelled_path.poses.push_back(rrt.plan_.poses[i]);
        }
        
        //Send whole trajectory - does not work good.
        /*
        trajectory_msgs::MultiDOFJointTrajectory Dji_reference_trajectory_;
        Dji_reference_trajectory_ = generate_trajectory_from_plan(rrt.plan_);
        rrt.getTrajectoryPub().publish(Dji_reference_trajectory_);
        */
      }
    }
    else
    {
      ROS_WARN_THROTTLE(5, "rrtplanner does not have a valid state yet!");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_WARN("Exiting rrtplanner...");
  ros::shutdown();
  return 0;
}
