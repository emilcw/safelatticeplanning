#pragma once

#include "ACADODjiMatrice.h"

bool GenerateMotionPrimitive(Trajectory& traj, State from, State to, double sampling_time, double finalTime, double& opt_cost);
