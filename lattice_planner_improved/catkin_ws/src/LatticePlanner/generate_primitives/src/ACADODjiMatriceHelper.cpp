#include "ACADODjiMatriceHelper.h"

bool createContiniousTrajectory(Trajectory& outputTrajectory, VariablesGrid& states,
                                double finalTime, int numberOfPieces)
{

	double timePerPiece = finalTime / ((double)numberOfPieces);

	State currentState = State();

    for (int i = 0; i <= numberOfPieces; i++)
	{
        currentState.time = i*timePerPiece;
        currentState.vx = states[i](0, vx_ind);
        currentState.vy = states[i](0, vy_ind);
        currentState.vz = states[i](0, vz_ind);

        currentState.x = states[i](0, x_ind);
        currentState.y = states[i](0, y_ind);
        currentState.z = states[i](0, z_ind);

        currentState.roll  = states[i](0,roll_ind);
        currentState.pitch = states[i](0,pitch_ind);
        currentState.yaw   = states[i](0,yaw_ind);

        currentState.roll_ref  = states[i](0,roll_ref_ind);
        currentState.pitch_ref = states[i](0,pitch_ref_ind);
        currentState.thrust    = states[i](0,thrust_ind);

		outputTrajectory.stateList.push_back(currentState);

	}

	return true;
}
