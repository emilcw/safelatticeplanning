#include "MotionPrimitiveGeneration.h"

bool GenerateMotionPrimitive(Trajectory& traj, State from, State to, double sampling_time, double finalTime, double& opt_cost)
{

    int nSegments = static_cast<int>(finalTime/sampling_time);

    return(ACADODjiMatrice(traj, from, to, nSegments, finalTime, opt_cost));
}
