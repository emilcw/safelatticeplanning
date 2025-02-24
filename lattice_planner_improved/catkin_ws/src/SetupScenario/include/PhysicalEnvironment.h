#pragma once
#ifndef __PHYSICAL_ENVIRONMENT_H__
#define __PHYSICAL_ENVIRONMENT_H__

#include <vector>
#include "Obstacle.h"
#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
//#include "State.h"
//#include "TrajectoryState.h"


//TODO: Remove?
//#include "MotionPrimitive.h"
#include "UAV/MotionPrimitive.h"
using MotionPrimitive = UAV::MotionPrimitive;

//TODO: Remove?
//#include "State.h"
#include "UAV/State.h"
#include "Search/Search.h"
//#include "TrajectoryState.h"
using TrajectoryState = UAV::TrajectoryState;
using State = search::SearchState<UAV::SearchState>;
using SearchState = UAV::SearchState;
using namespace octomap;


class PhysicalEnvironment {
  private:
    AABB allowed_volume;
    std::vector<Obstacle> static_obstacles;
    std::vector<Obstacle> dynamic_obstacles;

    std::vector<MotionPrimitive> primitives;
    std::vector<std::vector<int> > primary_primitive_to_primitive_map;  // primary -> primary
    std::vector<std::vector<int> > secondary_primitive_to_primitive_map; // primary -> secondary and secondary -> secondary
    int secondary_primitives_start_index;

    float VELOCITY_DIFFERANCE_TOLERANCE = 0.1;
    octomap::OcTree* octree;

  public:

    /* Obstacles */
    std::vector<Obstacle> get_obstacles();
    std::vector<Obstacle> & get_static_obstacles() { return static_obstacles; }
    std::vector<Obstacle> & get_dynamic_obstacles() { return dynamic_obstacles; }
    void update_octomap(octomap::OcTree* octree);
    void add_obstacle(Obstacle obstacle);
    void clear_predictions();
    void clear_obstacles() { static_obstacles.clear(); dynamic_obstacles.clear(); }

    /* Motion primitives */
    bool load_primitives(std::string path,
                         std::vector<int> primitive_amount_in_group,
                         bool use_stand_still,
                         bool use_secondary_primitives,
                         bool use_only_secondary_primitives);
    std::vector<MotionPrimitive> & get_primitives() { return primitives; }
    MotionPrimitive & get_primitive(int index) { return primitives[index]; }
    std::vector<std::vector<int> > & get_primary_primitive_map() { return primary_primitive_to_primitive_map; }
    std::vector<std::vector<int> > & get_secondary_primitive_map() { return secondary_primitive_to_primitive_map; }
    int & get_secondary_start_index() { return secondary_primitives_start_index; }
    void calculate_motion_primitive_hitboxes(Hitbox & vehicle);

    /* Allowed environment bound */
    AABB & get_allowed_volume() { return allowed_volume; }
    void set_allowed_volume(AABB allowed_volume) { this->allowed_volume = allowed_volume; }
    void set_allowed_volume(double minX, double maxX,
                          double minY, double maxY,
                          double minZ, double maxZ);
    bool in_allowed_volume(State * state);

    /* Collision checking */
    double evaluate_proximity_cost(Hitbox uav_hitbox, State * state, double start_time = 0.0, bool static_only = false);
    bool in_collision(Hitbox uav_hitbox, State * state, bool static_only = false, double start_time = 0.0);
    bool in_collision(Hitbox uav_hitbox, std::vector<State*> & path, double start_time = 0.0);
    bool in_collision(Hitbox uav_hitbox, std::vector<State> & path, double start_time = 0.0);
    bool in_collision(Hitbox uav_hitbox, std::vector<TrajectoryState> & trajectory, double start_time = 0.0);

    /* Debugging */
    void print_primitive_map();
    void print_obstacles();
    bool debug_primitives(bool abort_on_errors = false);

  private:
    std::vector<MotionPrimitive> load_primitives(std::string path, std::vector<int> primitive_amount_in_group, bool use_stand_still);
    std::vector<MotionPrimitive> load_secondary_primitives(std::string path, double cost_multiplier);
    std::vector<std::vector<int> > build_adjacency_map(std::vector<MotionPrimitive> & primitives,
                                                       double resolution);

};


/* Debug utility */
template <class T>
std::ostream& operator<<(std::ostream& os, std::vector<T> & o) {
  os << "[";
  if(!o.empty()) {
    for(int n = 0; n < o.size()-1; n++)
      os << o[n] << ",";
    os << o.back();
  }
  os << "]";
  return os;
}

#endif
