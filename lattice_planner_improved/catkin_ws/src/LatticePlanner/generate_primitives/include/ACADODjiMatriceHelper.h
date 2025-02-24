#pragma once

#include "Trajectory.h"

#define PRINT_PROGRESS 1


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#define vx_ind 0
#define vy_ind 1
#define vz_ind 2

#define roll_ind 3
#define pitch_ind 4
#define yaw_ind 5

#define x_ind 6
#define y_ind 7
#define z_ind 8

#define roll_ref_ind 9
#define pitch_ref_ind 10
#define thrust_ind 11

#define roll_ref_dot_ind 12
#define pitch_ref_dot_ind 13
#define thrust_dot_ind 14

USING_NAMESPACE_ACADO

bool createContiniousTrajectory(Trajectory& outputTrajectory, VariablesGrid& states, double finalTime, int numberOfPieces);
