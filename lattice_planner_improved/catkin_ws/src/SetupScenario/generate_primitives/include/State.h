#pragma once
#ifndef __STATE_H__
#define __STATE_H__

#include <list>
#include <math.h>
#include <iostream>

using namespace std;

class State
{
public:
    //States
    double x, y, z;	                // Position
    double vx, vy, vz;              // Velocity
    double roll, pitch, yaw;        // Orientation

    //Controls
    double roll_ref, pitch_ref;
    double thrust;

    //Extra state for acado
    double roll_ref_dot;
    double pitch_ref_dot;
    double thrust_dot;

    //time on traj
    double time;

	State();
	State(const State& stateIn);
    State(double x_in, double y_in, double z_in, double vx_in, double vy_in, double vz_in,
          double roll_in, double pitch_in, double yaw_in, double roll_ref_in, double pitch_ref_in,
          double thrust_in, double roll_ref_dot_in, double pitch_ref_dot_in, double thrust_dot_in, double time);

	void operator=(const State& other);
	friend State operator+(State lhs,        // passing lhs by value helps optimize chained a+b+c
                 const State& rhs);
  
  void print();
};

#endif //__STATE_H__
