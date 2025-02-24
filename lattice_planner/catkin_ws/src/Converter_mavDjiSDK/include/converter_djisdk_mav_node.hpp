#pragma once

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <assert.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
//include "mav_msgs/eigen_mav_msgs.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
//#include <mav_msgs/default_topics.h>
#include "nav_msgs/Odometry.h"
#include <mav_msgs/common.h>
