#pragma once
#ifndef __SCENARIO_H__
#define __SCENARIO_H__

#include <vector>
#include "geometry_msgs/Point.h"
#include "Obstacle.h"

//TODO: Remove?
//#include "State.h"
#include "UAV/State.h"
#include "Search/Search.h"
using State = search::SearchState<UAV::SearchState>;

class Scenario {
  public:
    Scenario(string name = "undefined", double duration = 0, AABB limits=AABB()) : name(name), duration(duration), limits(limits) {}
    ~Scenario() {}

  public:
    string name;
    std::vector<Obstacle> obstacles;
    
    geometry_msgs::Point start;
    geometry_msgs::Point goal;
    
    int seed;

    // Randomized scenarios
    bool randomize_goal = false;
    double duration = 0;
    AABB limits;
};

Scenario create_scenario(std::string scenario_name, std::string test_type, double parameter, int seed = 0, bool predictable = true);


/* Utilities */
inline geometry_msgs::Point create_point(double x, double y, double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}
inline Obstacle create_CV_obstacle(SearchState state, double radius) {
  StateSpaceModel * model = new ConstantVelocityModel2(state);
  Obstacle obstacle(state.position.x(), state.position.y(), state.position.z(), radius, "constantVelocity", model);
  return obstacle;
}

inline Obstacle create_advanced_obstacle(double x, double y, double z, double maxSpeed, std::string type, AABB limit, bool predictable = true, int id = 0) {
    float radius = 1.0;
    StateSpaceModel * model = new ConstantVelocityModel2(SearchState(x, y, z, 0, 0, 0), radius);
    Obstacle advanced( x, y, z, radius, type, model, 1.0/10.0, id);
    advanced.is_advanced = true;
    advanced.type = type;
    advanced.max_speed = maxSpeed;
    advanced.limit = limit;
    
    if(!predictable){
      advanced.predictable = false;
    }    
    return advanced;
}


#endif
