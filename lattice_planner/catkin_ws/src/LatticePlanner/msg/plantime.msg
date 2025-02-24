Header header

# The time for which the plan is valid (Actual state time: trajectory[n].time + plan_start_time)
#time plan_start_time

uint32[] primitive_id
duration[] time_from_start
float64[] durations
geometry_msgs/Point[] initial_point

#uint secondary_trajectory_index
#trajectory_msgs/MultiDOFJointTrajectory trajectory
