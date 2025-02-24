#include "Obstacle.h"

/*
NOTE: Currently not used!
Checks if state is in collision with obstacle.
*/
bool Obstacle::collision_with_state(const State & state, double uav_safety_radius)
{
  Hitbox uav_hitbox = Hitbox(state.state.position.x(), state.state.position.y(), state.state.position.z(), uav_safety_radius);
  return uav_hitbox.intersecting(this->obstacle_hitbox);
}


/*
NOTE: Currently not used
Checks if traj is in collision with obstacle.
*/
bool Obstacle::collision_with_trajectory(const std::vector<TrajectoryState> & traj, double uav_safety_radius)
{
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


/*
Return the Obstacle (this) hitbox.
*/
Hitbox & Obstacle::get_hitbox() 
{
  return obstacle_hitbox;
}


/*
Type: Static and Dynamic

Compute the prediction of the Obstcle in a conservative way and compute
the distance between bouding boxes. Insert this value into the weird cost_function
to get a proper proximity_cost.
*/
double Obstacle::proximity_cost(Hitbox & hitbox, double time, bool conservative) 
{
  double min_dist = std::numeric_limits<float>::infinity(); //Default value (not-collision)
  const Hitbox & prediction = predict(time, conservative);  //Conservative Prediction
  double dist = hitbox.distance_spheres(prediction);        //Return distance between spheres (not from center but from radius)
  if (dist < min_dist) min_dist = dist;
  return this->cost_func(min_dist);                         //dist > 0 ? 0.1*exp(-2*dist*dist) : std::numeric_limits<float>::infinity(); 
}


/*
Type: Static and Dynamic
Sum over the entire Motion Primitive to see if there is collision or not.
Compute the proximity cost for each TrajectoryState in Motion Primitive.
*/
double Obstacle::proximity_cost(std::vector<TrajectoryState> & traj, Hitbox & hitbox, double start_time, bool conservative) 
{
  double traj_cost = 0;  // Sum cost over trajectory state 
  for(int n = 0; n < traj.size(); n++) 
  {
    if(traj[n].time < start_time) continue;
    hitbox.set_position(traj[n].position.x(), traj[n].position.y(), traj[n].position.z());
    traj_cost += proximity_cost(hitbox, traj[n].time, conservative);
  }  
  return traj_cost;
}


/*
Type: Static Collision Checking

Collision check if the uav hitbox is is in intersection with the obstacle hitbox.
This applies only on static obstacles since we need time otherwise.

NOTE: The hitbox is either (1) the UAV hibox (R = 1) or (2) A motion primitive hitbox (R usually not 1)
*/
bool Obstacle::in_collision(Hitbox & hitbox) 
{
  return hitbox.intersecting(obstacle_hitbox);
}


/*
Type: Static and Dynamic Collision checking
This function creates a prediction of where and obstale will be a time <time>
and checks if there will be and intersection with the position of the given hitbox.

NOTE: It does the prediction in a Conservative way (bigger radius)
NOTE: The hitbox does not have to be the UAV hitbox, it can be any hitbox.

So either predict does not work or intersect
NOTE: The hitbox is either (1) the UAV hibox (R = 1) or (2) A motion primitive hitbox (R usually not 1)

*/
bool Obstacle::in_collision(Hitbox & hitbox, double time, bool conservative) 
{
  
  clear_predictions();
  
  const Hitbox & obstacle_prediction = predict(time, conservative);

  //Visualize the prediction
  if(hitbox.intersecting(obstacle_prediction)) 
  {
    return true;
  }
  return false;
}


/*
Type: Static and Dynamic
This function creats a prediction from <time> until <time + duration>
and checks if the corresponding Hitbox is in collision with the given hitbox.
*/
bool Obstacle::in_collision(Hitbox & hitbox, double time, double duration) 
{
  const Hitbox & prediction = predict(time, duration);
  if(hitbox.intersecting(prediction)) 
  {
    return true;
  }
  return false;
}


/*
Type: Static
Currently used in PhysicalEnvironment.cpp (Emil Wiman)

Checks if all states in Motion Primitive (vector<TrajectoryState>) is in collision with static environment or not.
If in collision, return true, the time of collision and the actual Trajectory State (collision-state).
*/
std::tuple<bool, double, TrajectoryState> Obstacle::in_collision_static(std::vector<TrajectoryState> & traj, Hitbox & hitbox, double start_time) 
{
  for(int n = 0; n < traj.size(); n++) 
  {
    if(traj[n].time < start_time) continue;
    hitbox.set_position(traj[n].position.x(), traj[n].position.y(), traj[n].position.z());
    if(in_collision(hitbox))
    {
      return std::make_tuple(true, traj[n].time, traj[n]);
    }
  }  
  return std::make_tuple(false, std::numeric_limits<float>::infinity(), TrajectoryState());
}


/*
Type: Dynamic
Currently used in PhysicalEnvironment.cpp (Emil Wiman)

Checks if all states in Motion Primitive (vector<TrajectoryState>) is in collision with dynamic environment or not.
If in collision, return true, the time of collision and the actual Trajectory State (collision-state).
*/
std::tuple<bool, double, TrajectoryState> Obstacle::in_collision(std::vector<TrajectoryState> & traj, Hitbox & hitbox, double start_time, bool conservative) 
{

  for(int n = 0; n < traj.size(); n++) 
  {
    if(traj[n].time < start_time)
    {
      continue;
    } 

    hitbox.set_position(traj[n].position.x(), traj[n].position.y(), traj[n].position.z());
    if(in_collision(hitbox, traj[n].time, conservative)) 
    {
      return std::make_tuple(true, traj[n].time, traj[n]);
    }
  }  
  return std::make_tuple(false, std::numeric_limits<float>::infinity(), TrajectoryState());
}


/*
Type: Static and Dynamic Collision Checking
This function returns a prediction of a given obstacle of where it might be a time <time>.
If conservative, we get a prediction with bigger radius

1. Check if static prediction
2. Check if we already have the prediction
3. Create the prediction (conservative/non-conservative)

*/
Hitbox & Obstacle::predict(double time, bool conservative) 
{
  bool dynamic = state != 0;
  
  //If static Obstacle
  if(!dynamic || !this->predictable) 
  {
    return obstacle_hitbox;
  }

  unsigned int time_n;
  time_n = std::max(0.0, std::round(time/dt));

  /*
  Removed since might contain not up do date predictions
  //If we have the required prediction
  if(predictions.size() > time_n) 
  {
    return predictions[time_n];
  }
  */
  
  float time_d = time_n * dt;
  Hitbox prediction;

  //If we don't have the prediction, create it (in a conservative or non conservative way)
  if(conservative) 
  {
    //Prediction is created with earlier start and longer duration (Creates bigger sphere!)
    //prediction = predict(time_d, dt);
    prediction = state->predict(time_d);
    prediction.r += (obstacle_hitbox.r * 1.1); //Tuning paramter on tradoff between commitment and safety.
  
  }
  else 
  {
    //Create prediction from motion model with dt = time_d
    //Override the radius with the obstacle radius.    
    prediction = state->predict(time_d);
    prediction.r += (obstacle_hitbox.r);
  }

  predictions.push_back(prediction);
  return predictions.back();
}


/*
Clear predictions vector connected to a certain Obstacle
Clear predictions map connected to a certain Obstacle
*/
void Obstacle::clear_predictions() 
{
  predictions.clear();
  predictions_map.clear();
}


/* 
Make a conservative prediction for where the object can be from <time> until <time + duration>.   
*/
Hitbox Obstacle::predict(double time, double time_duration) 
{
  bool dynamic = state != 0;
  
  //If static obstacle
  if(!dynamic || !this->predictable) 
  {
    return obstacle_hitbox;
  }
  
  //Otherwise, compute prediction future in time
  Hitbox prediction;
  
  //Compute first point of prediction and add proper radius
  Hitbox prediction_start = state->predict(time);
  prediction_start.r += obstacle_hitbox.r;

  //Compute end point of prediction and add proper radius
  Hitbox prediction_end = state->predict(time+time_duration);
  prediction_end.r += obstacle_hitbox.r;

  //Compute temporal hitbox
  //Position: Between prediction.start and prediction.end
  //Radius: The sphere will intersect with both prediction.start and prediction.end
  float distance = (prediction_start.position - prediction_end.position).norm();
  prediction.position = (prediction_start.position + prediction_end.position)/2;

  prediction.r = std::max(prediction_start.getR(),prediction_end.getR()) + distance / 2;
  
  return prediction;  
}
