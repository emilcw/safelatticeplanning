#pragma once
#ifndef __STATE_SPACE_MODEL_H__
#define __STATE_SPACE_MODEL_H__

#include "Hitbox.h"
#include "ros/ros.h"
#include "UAV/State.h"
#include "Search/Search.h"

using SearchState = UAV::SearchState;

class StateSpaceModel 
{

protected:
  double previous_time;

public:
  virtual ~StateSpaceModel() {};
  virtual Hitbox predict(double time) = 0;
  virtual void simulate(double time) = 0;
  virtual void observe_position(double time, Hitbox hitbox) = 0;

  void reset_time() { previous_time = ros::Time::now().toSec(); }
  void reset_time(double time) { previous_time = time; }
  
  double calculate_dt() 
  {
    return ros::Time::now().toSec() - previous_time;
  }
  
  double calculate_dt(double time) 
  {
    return time - previous_time;
  }

  SearchState & get_state() { return state; }

public:
  SearchState state;

};


/*

ConstantVelocityModel
NOTE: Radius it growing with increasing dt
Used as simple dynamic obstacle.

*/
class ConstantVelocityModel : public StateSpaceModel 
{
  public:
    ConstantVelocityModel(SearchState state = SearchState(0,0,0), double r = 0.0) : r(r) { this->state = state; }
    ~ConstantVelocityModel() {}


    /*
    Predict a states future position given a constant velocity motion model and a time step dt.
    If dt = 0, we only get the current observation (position and radius) of the obstacle.
    */
    Hitbox predict(double dt) override 
    {
      //Propagte position on step forward
      Hitbox prediction = Hitbox(state.position.x(), state.position.y(), state.position.z(), r);
      
      if(dt < 0)
      {
        ROS_ERROR_STREAM("ConstantVelocityModel: predict(): dt < 0 should not be possible, returns current position");
        return prediction;
      }

      prediction.position += (dt * state.velocity);

      //Find max of vel_x, vel_y and vel_z
      double max_velocity = std::max( std::max( std::abs(state.velocity.x()), std::abs(state.velocity.y()) ), std::abs(state.velocity.z()));
      
      //Propagate radius on step forward
      prediction.r += (dt * max_velocity * 0.5);

      return prediction;
    }

    /*
    Actually simulate (move) a dynamic obstacle state with a constant velocity motion model.
    */
    void simulate(double time) override 
    {
      double dt = calculate_dt(time);
      state.position += dt * state.velocity;
      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
      r += dt * max_velocity * 0.5;      
    }

    /*
    Force dynamic obstacle state (position and radius) to be the same as for hitbox (where hitbox usually comes from predict())
    */
    void observe_position(double time, Hitbox hitbox) override 
    {
      state.position = hitbox.getPosition();
      r = hitbox.getR();
      reset_time(time);
    }

  private:
    double r;
};


/*

ConstantVelocityModel2
NOTE: Radius is *NOT* growing with increasing dt
Used add_advanced_obstacle.

*/
class ConstantVelocityModel2 : public StateSpaceModel 
{
  public:
    ConstantVelocityModel2(SearchState state = SearchState(0,0,0), double r = 0.0) : r(r) { this->state = state; }
    ~ConstantVelocityModel2() {}

    Hitbox predict(double dt) override 
    {
      Hitbox prediction = Hitbox(state.position.x(), state.position.y(), state.position.z(), r);

      if(dt < 0)
      {
        ROS_ERROR_STREAM("ConstantVelocityModel2: predict(): dt < 0 should not be possible, returns current position");
        return prediction;
      }

      prediction.position += (dt * state.velocity);
      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
      
      return prediction;
    }

    void simulate(double time) override 
    {
      double dt = calculate_dt(time);
      state.position += dt * state.velocity;
      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
    }

    void observe_position(double time, Hitbox hitbox) override 
    {
      state.position = hitbox.getPosition();
      r = hitbox.getR();
      reset_time(time);
    }

  private:
    double r;
};

#endif // __STATE_SPACE_MODEL_H__
