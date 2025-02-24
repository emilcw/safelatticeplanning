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
  if(!dynamic || !this->predictable) 
  {
    return obstacle_hitbox;
  }

  unsigned int time_n;
  time_n = std::max(0.0, std::round(time/dt));
  if(predictions.size() > time_n) 
  {
    return predictions[time_n];
  }
  
  float time_d = time_n * dt;
  Hitbox prediction;
  if(conservative) 
  {

    //prediction = predict(time_d-0.5*dt, 1.5*dt); //This one is alwayws 2.5
  
    prediction = state->predict(time_d); // This prediction.r = 0
    prediction.r += (obstacle_hitbox.r * 1.1); //Tuning paramter on tradoff between commitment and safety.
  }
  else 
  {

    prediction = state->predict(time_d); // This prediction.r = 0
    prediction.r += obstacle_hitbox.r; // Here it becomes 1
  }


  predictions.push_back(prediction);
  return predictions.back();
}

void Obstacle::clear_predictions() {
  predictions.clear();
  predictions_map.clear();
}

/* 
Make a conservative prediction for where the object can be from <time> until <time + duration>.   
*/
Hitbox Obstacle::predict(double time, double time_duration) {
  
  bool dynamic = state != 0;
  if(!dynamic || !this->predictable) 
  {
    return obstacle_hitbox;
  }
  
  //Empty prediction with radius 1
  Hitbox prediction;

  //Start point
  Hitbox prediction_start = state->predict(time);
  prediction_start.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction

  //End point
  Hitbox prediction_end = state->predict(time+time_duration);
  prediction_end.r += obstacle_hitbox.r; // Add volume of obstacle to position prediction

  //Calculate prediction hitbox (temporal hitbox)
  float distance = (prediction_start.position - prediction_end.position).norm();
  prediction.position = (prediction_start.position + prediction_end.position)/2;
  prediction.r = std::max(prediction_start.getR(), prediction_end.getR()) + (distance/2);
  
  return prediction;  
}