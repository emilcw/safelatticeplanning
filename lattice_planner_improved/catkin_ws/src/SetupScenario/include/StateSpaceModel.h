#pragma once
#ifndef __STATE_SPACE_MODEL_H__
#define __STATE_SPACE_MODEL_H__

#include "Hitbox.h"
#include "ros/ros.h"

//TODO: Remove?
//#include "SearchState.h"
#include "UAV/State.h"
#include "Search/Search.h"
using SearchState = UAV::SearchState;

class StateSpaceModel {
protected:
  double previous_time;

public:
  virtual ~StateSpaceModel() {};
  virtual Hitbox predict(double time) = 0;
  virtual void simulate(double time) = 0;
  virtual void observe_position(double time, Hitbox hitbox) = 0;

  void reset_time() { previous_time = ros::Time::now().toSec(); }
  void reset_time(double time) { previous_time = time; }
  
  double calculate_dt() {
    return ros::Time::now().toSec() - previous_time;
  }
  double calculate_dt(double time) {
    return time - previous_time;
  }

  SearchState & get_state() { return state; }

public:
  SearchState state;

};


/*
ConstantVelocityModel

* Growding radius with each time-step
* Mainly used for simple dynamic obstacles.

*/
class ConstantVelocityModel : public StateSpaceModel {
  public:
    ConstantVelocityModel(SearchState state = SearchState(0,0,0), double r = 0.0) : r(r) { this->state = state; }
    ~ConstantVelocityModel() {}

    Hitbox predict(double dt) override {
      Hitbox prediction = Hitbox(state.position.x(), state.position.y(), state.position.z(), r);
      prediction.position += (dt * state.velocity);

      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
      
   
      prediction.r += (dt * max_velocity * 0.5);

      return prediction;
    }

    void simulate(double time) override {
      double dt = calculate_dt(time);
      state.position += dt * state.velocity;
      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
      r += dt * max_velocity * 0.5;      
    }

    void observe_position(double time, Hitbox hitbox) override {
      state.position = hitbox.getPosition();
      r = hitbox.getR();
      reset_time(time);
    }

  private:
    double r;
};

/*
ConstantVelocityModel2

* NOT having a growing radius
* Used by add_advanced_obstacle

*/
class ConstantVelocityModel2 : public StateSpaceModel {
  public:
    ConstantVelocityModel2(SearchState state = SearchState(0,0,0), double r = 0.0) : r(r) { this->state = state; }
    ~ConstantVelocityModel2() {}

    Hitbox predict(double dt) override {
      Hitbox prediction = Hitbox(state.position.x(), state.position.y(), state.position.z(), r);
      prediction.position += dt * state.velocity;
      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
      return prediction;
    }

    void simulate(double time) override {
      double dt = calculate_dt(time);
      state.position += dt * state.velocity;
      double max_velocity = std::max(std::max(std::abs(state.velocity.x()), std::abs(state.velocity.y())), std::abs(state.velocity.z()));
    }

    void observe_position(double time, Hitbox hitbox) override {
      state.position = hitbox.getPosition();
      r = hitbox.getR();
      reset_time(time);
    }

  private:
    double r;
};

#endif // __STATE_SPACE_MODEL_H__
