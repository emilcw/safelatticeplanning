#include "Obstacle.h"


bool Obstacle::collision_with_state(const State & state, double uav_safety_radius){
  Hitbox uav_hitbox = Hitbox(state.state.position.x(), state.state.position.y(), state.state.position.z(), uav_safety_radius);
  return uav_hitbox.intersecting(this->obstacle_hitbox);
}
  
bool Obstacle::collision_with_trajectory(const std::vector<TrajectoryState> & traj, double uav_safety_radius){
  int traj_length = traj.size();
  int jumping_index = std::max(1, traj_length/10); 
  for(int n = 0; n < traj.size(); n += jumping_index){
    Hitbox uav_hitbox = Hitbox(traj[n].position.x(),traj[n].position.y(),traj[n].position.z(),uav_safety_radius);
    if(uav_hitbox.intersecting(this->obstacle_hitbox))
      return true;
  }
  
  for(int n = 0; n < traj.size(); n++){
    Hitbox uav_hitbox = Hitbox(traj[n].position.x(),traj[n].position.y(),traj[n].position.z(),uav_safety_radius);
    if(uav_hitbox.intersecting(this->obstacle_hitbox))
      return true;
  }
  return false;
}  

Hitbox & Obstacle::get_hitbox() {
  return obstacle_hitbox;
}

double Obstacle::proximity_cost(Hitbox & uav, double time) {
  double min_dist = std::numeric_limits<float>::infinity();  // Return the worst over different quantizations of time
  const Hitbox & prediction = predict(time, true);
  double dist = uav.distance_spheres(prediction);
  if (dist < min_dist) min_dist = dist;
  return this->cost_func(min_dist);
}
/*
double Obstacle::proximity_cost(Hitbox & uav, double time) {
  double min_dist = std::numeric_limits<float>::infinity();  // Return the worst over different quantizations of time
  for(int k = 0; k < 2; k++) {
    const Hitbox & prediction = predict(time, k);
    double dist = uav.distance_spheres(prediction);
    if (dist < min_dist) min_dist = dist;
  }
  return this->cost_func(min_dist);
}
*/

double Obstacle::proximity_cost(std::vector<TrajectoryState> & traj, Hitbox & uav, double start_time) {
  double traj_cost = 0;  // Sum cost over trajectory state 
  for(int n = 0; n < traj.size(); n++) {
  //for(int n = 0; n < traj.size(); n += 5) {
    if(traj[n].time < start_time) continue;
    uav.set_position(traj[n].position.x(), traj[n].position.y(), traj[n].position.z());
    traj_cost += proximity_cost(uav, traj[n].time);
  }  
  return traj_cost;
}

bool Obstacle::in_collision(Hitbox & uav) {  // Static collision check
  return uav.intersecting(obstacle_hitbox);
}

bool Obstacle::in_collision(Hitbox & uav, double time) {
  const Hitbox & prediction = predict(time, false);
  //Hitbox prediction_worse(prediction.x, prediction.y, prediction.z, prediction.r*1.1);
  //if(uav.intersecting(prediction_worse)) {
  if(uav.intersecting(prediction)) {
    return true;
  }
  return false;
}
/*
bool Obstacle::in_collision(Hitbox & uav, double time) {
  for(int k = 0; k < 2; k++) {
    const Hitbox & prediction = predict(time, k);
    //Hitbox prediction_worse(prediction.x, prediction.y, prediction.z, prediction.r*1.1);
    //if(uav.intersecting(prediction_worse)) {
    if(uav.intersecting(prediction)) {
      return true;
    }
  }
  return false;
}*/

bool Obstacle::in_collision(Hitbox & uav, double time, double duration) {
  const Hitbox & prediction = predict(time, duration);
  //Hitbox prediction_worse(prediction.x, prediction.y, prediction.z, prediction.r*1.1);
  //if(uav.intersecting(prediction_worse)) {
  if(uav.intersecting(prediction)) {
    return true;
  }
  return false;
}

bool Obstacle::in_collision_static(std::vector<TrajectoryState> & traj, Hitbox & uav, double start_time) {
  for(int n = 0; n < traj.size(); n++) {
    if(traj[n].time < start_time) continue;
    uav.set_position(traj[n].position.x(), traj[n].position.y(), traj[n].position.z());
    if(in_collision(uav)) {
      return true;
    }
  }  
  return false;
}

bool Obstacle::in_collision(std::vector<TrajectoryState> & traj, Hitbox & uav, double start_time) {
  for(int n = 0; n < traj.size(); n++) {
  //for(int n = 0; n < traj.size(); n += 5) {
    if(traj[n].time < start_time) continue;
    uav.set_position(traj[n].position.x(), traj[n].position.y(), traj[n].position.z());
    if(in_collision(uav, traj[n].time)) {
      return true;
    }
  }  
  return false;
}


Hitbox & Obstacle::predict(double time, bool conservative) {
  bool dynamic = state != 0;
  if(!dynamic || !this->predictable) {
    return obstacle_hitbox;
  }
  // Test: Do not use predictions
  //return obstacle_hitbox;

  unsigned int time_n;
  time_n = std::max(0.0, std::round(time/dt));
  if(predictions.size() > time_n) {
    return predictions[time_n];
  }
  
  float time_d = time_n * dt;
  //const bool interval_prediction = false;  // More costly, more conservative, more robust(?)
  Hitbox prediction;
  if(conservative) {
    prediction = predict(time_d-0.5*dt, 1.5*dt);
    //prediction = predict(time_d, 0);  // Same as state->predict(time_d)
  }
  else {
    prediction = state->predict(time_d);
    prediction.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction
  }

  predictions.push_back(prediction);
  return predictions.back();
}

void Obstacle::clear_predictions() {
  predictions.clear();
  predictions_map.clear();
}

/* Make a conservative prediction for where the object can be from
   time until time + duration.
   
   Warning: Assumes linear predictions, i.e. worst case radial translation! */
Hitbox Obstacle::predict(double time, double time_duration) {
  bool dynamic = state != 0;
  if(!dynamic || !this->predictable) {
    return obstacle_hitbox;
  }
  
  Hitbox prediction;
  Hitbox prediction_start = state->predict(time);
  prediction_start.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction
  Hitbox prediction_end = state->predict(time+time_duration);
  prediction_end.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction
  
  // calculate prediction hitbox (temporal hitbox)
  float distance = (prediction_start.position - prediction_end.position).norm();
  prediction.position = (prediction_start.position + prediction_end.position)/2;
  prediction.r = std::max(prediction_start.getR(),prediction_end.getR()) + distance/2;
  
    
  return prediction;  
}

/*
Hitbox & Obstacle::predict(double time, int n) {
  bool dynamic = state != 0;
  if(!dynamic || !this->predictable) {
    return obstacle_hitbox;
  }
  // Test: Do not use predictions
  //return obstacle_hitbox;

  unsigned int time_n;
  //unsigned int time_n = std::max(0.0, std::round(time/dt)); // or floor? or ceil?
  if(n == 0)
    time_n = std::max(0.0, std::ceil(time/dt));
  else if(n == 1)
    time_n = std::max(0.0, std::floor(time/dt));
  else
    time_n = std::max(0.0, std::round(time/dt));
  if(predictions.size() > time_n) {
    return predictions[time_n];
  }

  Hitbox prediction = state->predict(time);
  prediction.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction

  predictions.push_back(prediction);
  return predictions.back();
}
*/



/*
Hitbox & Obstacle::predict(double time) {
  bool dynamic = state != 0;
  if(!dynamic) {
    return obstacle_hitbox;
  }
  // Test: Do not use predictions
  //return obstacle_hitbox;

  std::map<double,Hitbox>::iterator prediction_iter = predictions_map.find(time);
  if(prediction_iter != predictions_map.end()) {
    return prediction_iter->second;
  }

  Hitbox & prediction = predictions_map[time];

  prediction = state->predict(time);
  prediction.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction

  return prediction;
}*/
