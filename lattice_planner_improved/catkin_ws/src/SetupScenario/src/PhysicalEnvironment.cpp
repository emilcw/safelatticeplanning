#include "PhysicalEnvironment.h"


void PhysicalEnvironment::add_obstacle(Obstacle obstacle) {
  if(obstacle.state == 0) {
    static_obstacles.push_back(obstacle);
  } else {
    dynamic_obstacles.push_back(obstacle);
  }
}

std::vector<Obstacle> PhysicalEnvironment::get_obstacles() {
  std::vector<Obstacle> obstacles(static_obstacles);
  obstacles.insert(obstacles.begin(), dynamic_obstacles.begin(), dynamic_obstacles.end());
  return obstacles;

}

void PhysicalEnvironment::clear_predictions() {
  for(std::vector<Obstacle>::iterator obstacle = dynamic_obstacles.begin();
      obstacle != dynamic_obstacles.end(); obstacle++) {
    obstacle->clear_predictions();
  }
}

void PhysicalEnvironment::set_allowed_volume(double minX, double maxX,
                                           double minY, double maxY,
                                           double minZ, double maxZ) {
  double rX = (maxX - minX)/2;
  double rY = (maxY - minY)/2;
  double rZ = (maxZ - minZ)/2;
  double cX =  minX + rX;
  double cY =  minY + rY;
  double cZ =  minZ + rZ;

  allowed_volume = AABB(cX, cY, cZ, rX, rY, rZ);
}

bool PhysicalEnvironment::in_allowed_volume(State * state) {
  return allowed_volume.is_inside(state->state.position.x(), state->state.position.y(), state->state.position.z());
}


bool PhysicalEnvironment::load_primitives(std::string path,
                                          std::vector<int> primitive_amount_in_group,
                                          bool use_stand_still,
                                          bool use_secondary_primitives,
                                          bool use_only_secondary_primitives) {
  double max_error = 0.01;                             // Option
  bool use_simple_secondary_search_adjacency = false; // Option
  primitives.clear();
  primary_primitive_to_primitive_map.clear();
  secondary_primitive_to_primitive_map.clear();

  /*
  ROS_INFO_STREAM("Loading primitives with parameters "
                  << "\n  path: " << path
                  << "\n  primitive_amount_in_group: " << primitive_amount_in_group
                  << "\n  use_stand_still: " << use_stand_still
                  << "\n  use_secondary_primitives: " << use_secondary_primitives
                  << "\n  use_only_secondary_primitives: " << use_only_secondary_primitives
                  << "\n  use_simple_secondary_search_adjacency: " << use_simple_secondary_search_adjacency
                  << "\n  max_error=: " << max_error);
                  */

  bool result = true;
  if(use_only_secondary_primitives) {
    primitives = load_secondary_primitives(path, 1.0);
    primary_primitive_to_primitive_map = build_adjacency_map(primitives, max_error);
    secondary_primitives_start_index = this->primitives.size();
    if(primitives.empty())
      result = false;
  }
  else {
    primitives = load_primitives(path, primitive_amount_in_group, use_stand_still);
    primary_primitive_to_primitive_map = build_adjacency_map(primitives, max_error);
    secondary_primitives_start_index = this->primitives.size();
    if(primitives.empty())
      result = false;
    if(use_secondary_primitives) {
      // Load and add secondary primitives
      std::vector<MotionPrimitive> secondary_primitives = load_secondary_primitives(path, 1.0);
      if(secondary_primitives.empty())
        result = false;
      // Append secondary primitives to the global primitive set
      primitives.insert(primitives.end(), secondary_primitives.begin(), secondary_primitives.end());
      // Build secondary search adjacency index structure
      std::vector<int> secondary_primitive_indecies;
      for(int n = 0; n < secondary_primitives.size(); n++) {
        secondary_primitive_indecies.push_back(n+secondary_primitives_start_index);
      }
      std::cerr << "secondary_primitive_indecies: " << secondary_primitive_indecies << "\n";

      if(use_simple_secondary_search_adjacency) {
        for(int n = 0; n < primitives.size(); n++) {
          secondary_primitive_to_primitive_map.push_back(secondary_primitive_indecies);
        }
      }
      else {
        // Add primary primitive to primary primitive transitions and primary to secondary transitions
        // ------------------------------------------------------
        for(int n = 0; n < secondary_primitives_start_index; n++) {
          // For each primary primitive
          MotionPrimitive & p1 = primitives[n];
          secondary_primitive_to_primitive_map.push_back(std::vector<int>());
          for(int k = 0; k < primary_primitive_to_primitive_map[n].size(); k++) {
            // For each neighbour of the primary primitive
            int p2_index = primary_primitive_to_primitive_map[n][k];
            MotionPrimitive & p2 = primitives[p2_index];
            // Add primary primitive to primary primitive transitions
            for(auto & secondary_primitive : secondary_primitives) {
              // For each secondary primitive
              if((p2.to.velocity - secondary_primitive.from.velocity).norm() <
                 (p1.to.velocity - secondary_primitive.from.velocity).norm()) {
                // Add as allowed transition if the velocity is strictly closer to any secondary primitive
                secondary_primitive_to_primitive_map[n].push_back(p2_index);
                break;
              }
            }
          }
        }
        // Add secondary primitive to secondary primitive transitions
        // ------------------------------------------------------
        for(int n = 0; n < secondary_primitives.size(); n++) {
          secondary_primitive_to_primitive_map.push_back(secondary_primitive_indecies);
        }
        // Add primary primitive to secondary primitivet transitions
        // ------------------------------------------------------
        for(int n = 0; n < secondary_primitives_start_index; n++) {
          // For each primary primitive
          MotionPrimitive & p1 = primitives[n];
          for(int k = secondary_primitives_start_index; k < primitives.size(); k++) {
            // For each secondary primitive
            MotionPrimitive & secondary_primitive = primitives[k];
            //if((p1.to.velocity - secondary_primitive.from.velocity).norm() < 1e-8) {
            if((p1.to.velocity - secondary_primitive.from.velocity).norm() < VELOCITY_DIFFERANCE_TOLERANCE) {
              // Add as allowed transition
              secondary_primitive_to_primitive_map[n].push_back(k);
            }
          }
        }
      }
    }
  }
  std::cerr << "secondary_primitive_to_primitive_map.size(): " << secondary_primitive_to_primitive_map.size() << "\n";

  int nPrimitives = primitives.size();
  int nPrimaryPrimitives = secondary_primitives_start_index;
  int nSecondaryPrimitives = nPrimitives - nPrimaryPrimitives;
  ROS_INFO_STREAM("Primitives loaded\n"
                  << "  #primitives:  " << nPrimitives << "\n"
                  << "    #primary:   " << nPrimaryPrimitives << "\n"
                  << "    #secondary: " << nSecondaryPrimitives);
  return result;
}

std::vector<MotionPrimitive> PhysicalEnvironment::load_primitives(std::string path, std::vector<int> primitive_amount_in_group, bool use_stand_still) {
  std::vector<MotionPrimitive> primitives;
  int start = (use_stand_still == 0);
  bool success;
  int startID = 0;
  for(int group = start; group < primitive_amount_in_group.size(); group++) {
    for(int n = 0; n < primitive_amount_in_group[group]; n++) {
      primitives.push_back(MotionPrimitive());
      MotionPrimitive & mp = primitives.back();
      std::string primitive_full_path = path +string("/") + string("primitive_group_") + to_string(group) + string("_id_") + to_string(n) + string(".txt");
      success = mp.load_from_file(primitive_full_path, startID);
      if(!success) {
        ROS_FATAL_STREAM("Could not load primitive '" << primitive_full_path << "'");
        return std::vector<MotionPrimitive>();
      }
    }
    startID += primitive_amount_in_group[group-start];
  }

  if(use_stand_still) {
    primitives.front().wait_time = 1;
    primitives.front().cost = 1;
  }

  return primitives;
}

std::vector<MotionPrimitive> PhysicalEnvironment::load_secondary_primitives(std::string path, double cost_multiplier) {
  std::vector<MotionPrimitive> primitives;
  int group = 8;
  bool success;
  for(int n = 0; n < 10; n++) {
    primitives.push_back(MotionPrimitive());
    MotionPrimitive & mp = primitives.back();
    std::string primitive_full_path = path +string("/") + string("primitive_group_") + to_string(group) + string("_id_") + to_string(n) + string(".txt"); // 0.5m resolution
    //std::string primitive_full_path = path +string("/") + string("primitive_group_") + to_string(group) + string("_id_") + to_string(n) + string(".txt"); // 1m resolution
    success = mp.load_from_file(primitive_full_path, 0);
    mp.cost *= cost_multiplier;
    mp.type = MotionPrimitive::SECONDARY;
    if(!success) {
      ROS_FATAL_STREAM("Could not load primitive '" << primitive_full_path << "'");
      return std::vector<MotionPrimitive>();
    }
  }
  return primitives;
}

std::vector<std::vector<int> > PhysicalEnvironment::build_adjacency_map(std::vector<MotionPrimitive> & primitives,
                                                                        double resolution) {
  std::vector<std::vector<int> > primitive_adjacency_index;
  for(int pIndex = 0; pIndex < primitives.size(); pIndex++) {
    primitive_adjacency_index.push_back(std::vector<int>());
    for(int n = 0; n < primitives.size(); n++) {
      if(std::abs(primitives[pIndex].to.velocity.x() - primitives[n].from.velocity.x()) < resolution &&
         std::abs(primitives[pIndex].to.velocity.y() - primitives[n].from.velocity.y()) < resolution &&
         std::abs(primitives[pIndex].to.velocity.z() - primitives[n].from.velocity.z()) < resolution) {
        primitive_adjacency_index.back().push_back(n);
      }
    }
  }
  return primitive_adjacency_index;
}

void PhysicalEnvironment::calculate_motion_primitive_hitboxes(Hitbox & vehicle) {
  for(MotionPrimitive & primitive : primitives) {
    primitive.hitbox = primitive.calculate_hitbox(vehicle);
  }
}

double PhysicalEnvironment::evaluate_proximity_cost(Hitbox uav_hitbox, State * state, double start_time, bool static_only) {

  if(!in_allowed_volume(state)) {
    return std::numeric_limits<double>::infinity();
  }

  Hitbox hitbox = Hitbox(state->state.position.x(),
                         state->state.position.y(),
                         state->state.position.z(),
                         uav_hitbox.getR());


  vector<double> resultsValue;
  if(octree !=NULL) {
    double radius = 1.0;
    point3d pmin = point3d(state->state.position.x()-radius,
                           state->state.position.y()-radius,
                           state->state.position.z()-radius);
    point3d pmax = point3d(state->state.position.x()+radius,
                           state->state.position.y()+radius,
                           state->state.position.z()+radius);
    for(OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(pmin,pmax),
          end=octree->end_leafs_bbx(); it!= end; ++it)
    {
      resultsValue.push_back(it->getOccupancy());
    }

  }
  auto occupancy = max_element(resultsValue.begin(),resultsValue.end());
  if (occupancy != resultsValue.end() && *occupancy > 0.5){
    return std::numeric_limits<double>::infinity();
  }

  start_time = 0;
  // Static obstacles, hard constraint
  // ==========================================================================
  if(state->parent == 0 || state->actionID < 0) { // To allow collision check on the root node
    for(Obstacle & obstacle : static_obstacles) {
      // Check new state *point* collision
      if(obstacle.in_collision(hitbox)) {
        // Vehicle is in a definite collision with an obstacle at the start position of the state primitive
        return std::numeric_limits<double>::infinity();
      }
    }
  }
  else {
    MotionPrimitive & primitive = get_primitive(state->actionID);
    Hitbox motion_primitive_hitbox = primitive.get_hitbox(state->state);
    std::vector<TrajectoryState> trajectory = primitive.apply((state->parent)->state, state->parent->time);
    float duration = primitive.duration;

    for(Obstacle & obstacle : static_obstacles) {
      // Check motion primitive temporal collision (conservative)
      if(obstacle.in_collision(motion_primitive_hitbox, state->parent->time)) {
        // Vehicle is in a possible collision with an obstacle during the execution of the state primitive

        // Check motion primitive *trajectory to state* collisions
        if(obstacle.in_collision(trajectory, hitbox, start_time)) {
          // Vehicle is in a definite collision with an obstacle during the execution of the state primitive
          return std::numeric_limits<float>::infinity();
        }
      }
    }
  }

  if(static_only)
    return false;

  double cost = 0;
  // Dynamic obstacles
  // ==========================================================================
  if(state->parent == 0 || state->actionID < 0) { // To allow collision check on the root node
    for(Obstacle & obstacle : dynamic_obstacles) {
      // Check new state *point* collision
      // Dynamic obstacle, soft constraints

      cost += obstacle.proximity_cost(hitbox, state->time);
    }
    //std::cerr << "------------------------------------------------------\n";
  }
  else {
    MotionPrimitive & primitive = get_primitive(state->actionID);
    Hitbox motion_primitive_hitbox = primitive.get_hitbox(state->state);
    std::vector<TrajectoryState> trajectory = primitive.apply((state->parent)->state, state->parent->time);
    float duration = primitive.duration;
    float radius = motion_primitive_hitbox.getR();

    for(Obstacle & obstacle : dynamic_obstacles) {
        // Extend radius to include soft constraint costs
        motion_primitive_hitbox.r = radius + obstacle.soft_constraint_radius; //TODO: if mp.hitbox.r is large, then there will be no meaningful *soft constraint* due to its calculation which is based on the distance to the mean position of the prediction.

        // Check motion primitive temporal collision (conservative)
        if(obstacle.in_collision(motion_primitive_hitbox, state->parent->time, duration)) {
          // Vehicle is in a possible collision with an obstacle during the execution of the state primitive

          // Dynamic obstacle, soft constraints
          cost += obstacle.proximity_cost(trajectory, hitbox, start_time);  // Can still be infinity on collisio
        }
    }
  }

  return cost;
}


/*
double PhysicalEnvironment::evaluate_proximity_cost(Hitbox uav_hitbox, State * state, double start_time, bool static_only) {

  if(!in_allowed_volume(state)) {
    return std::numeric_limits<double>::infinity();
  }

  Hitbox hitbox = Hitbox(state->state.position.x(),
                         state->state.position.y(),
                         state->state.position.z(),
                         uav_hitbox.getR());


  // Static obstacles
  // ==========================================================================
  // Check new state *point* collision(s)a
  for(std::vector<Obstacle>::iterator obstacle = static_obstacles.begin();
      obstacle != static_obstacles.end(); obstacle++) {
    // Static obstacle, hard constraint
    if(obstacle->in_collision(hitbox)) {
      return std::numeric_limits<double>::infinity();
    }
  }
  // Check motion primitive *trajectory to state* collision(s)
  if(state->parent != 0) {  // To allow collision check on the root node
    std::vector<TrajectoryState> trajectory =
      this->primitives[state->actionID].apply((state->parent)->state, state->parent->time);
    for(std::vector<Obstacle>::iterator obstacle = static_obstacles.begin();
        obstacle != static_obstacles.end(); obstacle++) {
      // Static obstacle, hard constraint
      if(obstacle->in_collision(trajectory, hitbox, start_time)) {
        return std::numeric_limits<float>::infinity();
      }
    }
  }

  if(static_only)
    return false;

  double cost = 0;
  // Dynamic obstacles
  // ==========================================================================
  // Check new state *point* collision(s)a
  for(std::vector<Obstacle>::iterator obstacle = dynamic_obstacles.begin();
      obstacle != dynamic_obstacles.end(); obstacle++) {
    // Dynamic obstacle, soft constraints
    cost += obstacle->proximity_cost(hitbox, state->time);  // Can still be infinity on collision
  }

  // Check motion primitive *trajectory to state* collision(s)
  if(state->parent != 0) {  // To allow collision check on the root node
    std::vector<TrajectoryState> trajectory =
      this->primitives[state->actionID].apply((state->parent)->state, state->parent->time);
    for(std::vector<Obstacle>::iterator obstacle = dynamic_obstacles.begin();
        obstacle != dynamic_obstacles.end(); obstacle++) {
      // Dynamic obstacle, soft constraints
      cost += obstacle->proximity_cost(trajectory, hitbox, start_time);  // Can still be infinity on collision
    }
  }
  return cost;

}
*/



// Hard in-collision (TODO: not needed?)
bool PhysicalEnvironment::in_collision(Hitbox uav_hitbox, State * state, bool static_only, double start_time) {
  if(!in_allowed_volume(state)) {
    return true;
  }

  Hitbox hitbox = Hitbox(state->state.position.x(),
                         state->state.position.y(),
                         state->state.position.z(),
                         uav_hitbox.getR());

  // Static obstacles
  // ==========================================================================
  /* Check if new state is in collision */
  for(std::vector<Obstacle>::iterator obstacle = static_obstacles.begin();
    obstacle != static_obstacles.end(); obstacle++) {
    if(obstacle->in_collision(hitbox)) {
      return true;
    }
  }
  /* Check if motion primitive trajectory is in collision */
  if(state->parent != 0) {  // To allow collision check on the root node
    std::vector<TrajectoryState> trajectory =
      this->primitives[state->actionID].apply((state->parent)->state, state->parent->time);
    for(std::vector<Obstacle>::iterator obstacle = static_obstacles.begin();
        obstacle != static_obstacles.end(); obstacle++) {
      if(obstacle->in_collision(trajectory, hitbox, start_time)) {
        return true;
      }
    }
  }
  if(static_only)
    return false;

  // Dynamic obstacles
  // ==========================================================================
  /* Check if new state is in collision */
  for(std::vector<Obstacle>::iterator obstacle = dynamic_obstacles.begin();
      obstacle != dynamic_obstacles.end(); obstacle++) {
    if(obstacle->in_collision(hitbox, state->time)) {
      return true;
    }
  }
  /* Check if motion primitive trajectory is in collision */
  if(state->parent != 0) {  // To allow collision check on the root node
    std::vector<TrajectoryState> trajectory =
      this->primitives[state->actionID].apply((state->parent)->state, state->parent->time);
    for(std::vector<Obstacle>::iterator obstacle = dynamic_obstacles.begin();
        obstacle != dynamic_obstacles.end(); obstacle++) {
      if(obstacle->in_collision(trajectory, hitbox, start_time)) {
        return true;
      }
    }
  }
  return false;
}

// Use only in re-planning to check that the previous plan is collision free
bool PhysicalEnvironment::in_collision(Hitbox uav_hitbox, std::vector<TrajectoryState> & trajectory, double start_time) {
  Hitbox hitbox = Hitbox(0,0,0,
                         uav_hitbox.getR());

  // Static obstacles
  for(std::vector<Obstacle>::iterator obstacle = static_obstacles.begin();
          obstacle != static_obstacles.end(); obstacle++) {
    if(obstacle->in_collision_static(trajectory, hitbox)) {
      return true;
    }
  }

  // Dynamic obstacles
  for(std::vector<Obstacle>::iterator obstacle = dynamic_obstacles.begin();
          obstacle != dynamic_obstacles.end(); obstacle++) {
    if(obstacle->in_collision(trajectory, hitbox, start_time)) {
      return true;
    }
  }
  return false;
}

bool PhysicalEnvironment::in_collision(Hitbox uav_hitbox, std::vector<State> & path, double start_time) {
  for(std::vector<State>::iterator state = path.begin(); state != path.end(); state++) {
    if(state->time <= start_time) continue;
    bool collision = in_collision(uav_hitbox, &(*state), start_time);
    if(collision) return true;
  }
  return false;
}

bool PhysicalEnvironment::in_collision(Hitbox uav_hitbox, std::vector<State*> & path, double start_time) {
  for(std::vector<State*>::iterator state = path.begin(); state != path.end(); state++) {
    if((*state)->time <= start_time) continue;
    bool collision = in_collision(uav_hitbox, *state, start_time);
    if(collision) return true;
  }
  return false;
}

void PhysicalEnvironment::print_primitive_map() {
  // Debug: Print secondary primitive adjacency
  std::cout << "Search1-primitives (primary primitives):\n";
  for(int pIndex = 0; pIndex < secondary_primitive_to_primitive_map.size(); pIndex++) {
    if(pIndex == secondary_primitives_start_index) {
      std::cout << "Search2-primitives (secondary primitives):\n";
    }
    std::cout << pIndex << "] ";
    for(int n = 0; n < secondary_primitive_to_primitive_map[pIndex].size(); n++) {
      std::cout << secondary_primitive_to_primitive_map[pIndex][n] << ", ";
    }
    std::cout << "\n";
  }
}

void PhysicalEnvironment::print_obstacles() {
  int n = 0;
  std::cout << "Printing static obstacles:\n";
  for(std::vector<Obstacle>::iterator obst = static_obstacles.begin(); obst != static_obstacles.end(); obst++) {
   cout << n << "] " << *obst << "\n";
    n++;
  }
  std::cout << "Printing dynamic obstacles:\n";
  for(std::vector<Obstacle>::iterator obst = dynamic_obstacles.begin(); obst != dynamic_obstacles.end(); obst++) {
   cout << n << "] " << *obst << "\n";
    n++;
  }
}

bool PhysicalEnvironment::debug_primitives(bool abort_on_errors) {
  std::vector<std::stringstream> reports;
  int errors = 0;


  // Check if primitives is empty
  if(primitives.empty()) {
    reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
    report << "[ERROR] No primitives loaded!\n";
    errors++;
  }

  // Check if each motion primitive is consistent wrt from-state, to-state and trajectory
  int index = 0;
  for(MotionPrimitive & primitive : primitives) {
    if(primitive.from != primitive.trajectory.front()) {
      reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
      report << "[ERROR] primitives[" << index << "].from != primitives[" << index << "].trajectory.front(): \n"
             << "  primitives[" << index << "]:                   " << primitive << "\n"
             << "  primitives[" << index << "].from:              " << primitive.from << "\n"
             << "  primitives[" << index << "].trajectory.back(): " << primitive.trajectory.back() << "\n";
      errors++;
    }
    if(primitive.to != primitive.trajectory.back()) {
      reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
      report << "[ERROR] primitives[" << index << "].to != primitives[" << index << "].trajectory.back(): \n"
             << "  primitives[" << index << "]:                   " << primitive << "\n"
             << "  primitives[" << index << "].from:              " << primitive.from << "\n"
             << "  primitives[" << index << "].trajectory.back(): " << primitive.trajectory.back() << "\n";
      errors++;
    }
  }

  // Check if primary 'primitive to primitive map' is consistent
  if(primary_primitive_to_primitive_map.empty()) {
    reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
    report << "[ERROR] primary_primitive_to_primitive_map is empty\n";
    errors++;
  }
  else {
    for(int index_from = 0; index_from < primary_primitive_to_primitive_map.size(); index_from++) {
      for(int index_to = 0; index_to < primary_primitive_to_primitive_map[index_from].size(); index_to++) {
        if(primary_primitive_to_primitive_map[index_from][index_to] < 0) {
          reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
          report << "[ERROR] primary_primitive_to_primitive_map[" << index_from << "][" << index_to << "] refers to a primitive index ("
                 << primary_primitive_to_primitive_map[index_from][index_to] << ") which does not exist!\n";
          errors++;
        } else
        if(primary_primitive_to_primitive_map[index_from][index_to] >= primitives.size()) {
          reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
          report << "[ERROR] primary_primitive_to_primitive_map[" << index_from << "][" << index_to << "] refers to a primitive index ("
                 << primary_primitive_to_primitive_map[index_from][index_to] << ") which does not exist! Size of primitives is "
                 << primitives.size() << "\n";
          errors++;
        } else {
          //Checks that make sure that only valid transitions exists.
          int index_p2 = primary_primitive_to_primitive_map[index_from][index_to];
          MotionPrimitive & p1 = get_primitive(index_from);
          MotionPrimitive & p2 = get_primitive(index_p2);
          if((p1.to.velocity - p2.from.velocity).norm() > VELOCITY_DIFFERANCE_TOLERANCE) {
            reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
            report << "[ERROR] primitive to primitive map :: primitives[" << index_from << "].from.velocity != primitives[" << index_p2 << "].to.velocity: \n"
                   << "   primitives[" << index_from << "]:      Name: " << p1.name << ", Group: " << p1.group << "\n"
                   << "   primitives[" << index_p2 <<   "]:      Name: " << p2.name << ", Group: " << p2.group << "\n"
                   << "   primitives[" << index_from << "].to:   " << p1.to << "\n"
                   << "   primitives[" << index_p2 <<   "].from: " << p2.from << "\n";
            errors++;
          }

          // Check if a primary transition referes to a secondary primitive
          if(index_p2 >= secondary_primitives_start_index || index_p2 == 279) {
            reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
            if(index_p2 == 279)
              report << "[ERROR] [ERROR] [ERROR] index_p2 == 279\n";
            report << "[ERROR] primitive to primitive map :: primitives[" << index_from << "] maps to [" << index_p2 << "], which is a SECONDARY PRIMITIVE: \n"
                   << "   primitives[" << index_from << "]:      Name: " << p1.name << ", Group: " << p1.group << "\n"
                   << "   primitives[" << index_p2 <<   "]:      Name: " << p2.name << ", Group: " << p2.group << "\n"
                   << "   primitives[" << index_from << "].to:   " << p1.to << "\n"
                   << "   primitives[" << index_p2 <<   "].from: " << p2.from << "\n"
                   << "   primary_primitive_to_primitive_map[" << index_from << "]: \n";
            int count = 0;
            for(int index : primary_primitive_to_primitive_map[index_from]) {
              report << "    " << count << "] " << index << "\n";
            }
            errors++;
            abort();
          }
        }
      }
    }
  }
  // Check if secondary 'primitive to primitive map' is consistent
  if(primary_primitive_to_primitive_map.empty()) {
    reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
    report << "[WARNING] secondary_primitive_to_primitive_map is empty\n";
  }
  else {
    for(int index_from = 0; index_from < secondary_primitive_to_primitive_map.size(); index_from++) {
      for(int index_to = 0; index_to < secondary_primitive_to_primitive_map[index_from].size(); index_to++) {
        if(secondary_primitive_to_primitive_map[index_from][index_to] < 0) {
          reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
          report << "[ERROR] secondary_primitive_to_primitive_map[" << index_from << "][" << index_to << "] refers to a primitive index ("
                 << secondary_primitive_to_primitive_map[index_from][index_to] << ") which does not exist!\n";
          errors++;
        } else
        if(secondary_primitive_to_primitive_map[index_from][index_to] >= primitives.size()) {
          reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
          report << "[ERROR] secondary_primitive_to_primitive_map[" << index_from << "][" << index_to << "] refers to a primitive index ("
                 << secondary_primitive_to_primitive_map[index_from][index_to] << ") which does not exist! Size of primitives is "
                 << primitives.size() << "\n";
          errors++;
        } else {
          //TODO: Add checks that make sure that only valid transitions exists. Do the same for secondary_primitive_to_primitive_map!
          int index_p2 = secondary_primitive_to_primitive_map[index_from][index_to];
          MotionPrimitive & p1 = get_primitive(index_from);
          MotionPrimitive & p2 = get_primitive(index_p2);
          if((p1.to.velocity - p2.from.velocity).norm() > VELOCITY_DIFFERANCE_TOLERANCE) {
            reports.push_back(std::stringstream()); std::stringstream & report = reports.back();
            report << "[ERROR] secondary primitive to primitive map :: primitives[" << index_from << "].from.velocity != primitives[" << index_p2 << "].to.velocity: \n"
                   << "   primitives[" << index_from << "]:      Name: " << p1.name << ", Group: " << p1.group << "\n"
                   << "   primitives[" << index_p2 <<   "]:      Name: " << p2.name << ", Group: " << p2.group << "\n"
                   << "   primitives[" << index_from << "].to:   " << p1.to << "\n"
                   << "   primitives[" << index_p2 <<   "].from: " << p2.from << "\n";
            errors++;
          }
        }
      }
    }
  }

  int MAX_REPORTS = 10;
  if(!reports.empty()) {
    if(errors > 0) {
      std::stringstream final_report;
      for(int n = 0; n < std::min(MAX_REPORTS,(int)reports.size()); n++) {
        final_report << " " << n << "] " << reports[n].str();
      }
      if(reports.size() > MAX_REPORTS) {
        final_report << "\n...\n(" << MAX_REPORTS << "/" << reports.size() << " errors shown)\n";
      }
      ROS_ERROR_STREAM("Motion Primitive Debug Report\n" <<
                       "=============================\n" <<
                       "Motion Primitive Debug Report\n" <<
                       "=============================\n" <<
                       "   " << errors << " errors found!\n" <<
                       "=============================\n" <<
                       "-----------------------------\n" <<
                       final_report.str().c_str() <<
                       "-----------------------------\n");
      if(abort_on_errors) {
        abort();
      }
    } else {
      std::stringstream final_report;
      for(int n = 0; n < reports.size(); n++) {
        final_report << " " << n << "] " << reports[n].str();
      }
      ROS_INFO_STREAM("Motion Primitive Debug Report\n" <<
                       "=============================\n" <<
                       "Motion Primitive Debug Report\n" <<
                       "-----------------------------\n" <<
                      final_report.str().c_str() <<
                      "-----------------------------\n");
    }
  }
  else {
    ROS_INFO_STREAM("Motion Primitive Debug Report\n" <<
                    "=============================\n" <<
                    "Motion Primitive Debug Report\n" <<
                    "-----------------------------\n" <<
                    "             OK              \n" <<
                    "-----------------------------\n");
  }
  return errors == 0;
}


void PhysicalEnvironment::update_octomap(octomap::OcTree* octree_){
  octree = octree_;
}
