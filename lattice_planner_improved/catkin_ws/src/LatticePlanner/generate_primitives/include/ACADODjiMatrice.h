#pragma once

#include "ACADODjiMatriceHelper.h"
#include "Trajectory.h"

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#define SUPRESS_ACADO_PRINT 1
//#define DEBUGGING_PLOT 1  

bool ACADODjiMatrice(Trajectory& outputTrajectory, State from, State to, int nSegments, double time, double& opt_cost);
