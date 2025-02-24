#include "ACADODjiMatrice.h"

bool ACADODjiMatrice(Trajectory& outputTrajectory, State from, State to, int nSegments, double finalTime, double& opt_cost)
{

    USING_NAMESPACE_ACADO

#if SUPRESS_ACADO_PRINT
	std::cout.setstate(std::ios_base::failbit);
#endif

    USING_NAMESPACE_ACADO

    double PI = 3.1415;

    DifferentialState velocity1; //velocity x_w
    DifferentialState velocity2; //velocity y_w
    DifferentialState velocity3; //velocity z_w

    DifferentialState roll;
    DifferentialState pitch;
    DifferentialState yaw;

    DifferentialState position1;
    DifferentialState position2;
    DifferentialState position3;

    DifferentialState roll_ref;
    DifferentialState pitch_ref;
    DifferentialState thrust;

    DifferentialState roll_ref_dot;
    DifferentialState pitch_ref_dot;
    DifferentialState thrust_dot;

    Control roll_ref_ddot;
    Control pitch_ref_ddot;
    Control thrust_ddot;

    Parameter  T;
    DifferentialEquation  f(0.0,T);
    //DifferentialEquation  f(0.0,finalTime);

    //Constants
    double roll_tau   = 0.253;
    double roll_gain  = 1.101;
    double pitch_tau  = 0.267;
    double pitch_gain = 1.097;
    double linear_drag_coefficient1 = 0.01;
    double linear_drag_coefficient2 = 0.01;
    double g = 9.8066;

    // initial conditions
    double velocity1_init = from.vx;
    double velocity2_init = from.vy;
    double velocity3_init = from.vz;
    double roll_init      = from.roll;
    double pitch_init     = from.pitch;
    double yaw_init       = from.yaw;
    double position1_init = from.x;
    double position2_init = from.y;
    double position3_init = from.z;
    double roll_ref_init  = from.roll_ref;
    double pitch_ref_init = from.pitch_ref;
    double thrust_init    = from.thrust;

    // final conditions
    double velocity1_end = to.vx;
    double velocity2_end = to.vy;
    double velocity3_end = to.vz;
    double roll_end      = to.roll;
    double pitch_end     = to.pitch;
    double yaw_end       = to.yaw;
    double position1_end = to.x;
    double position2_end = to.y;
    double position3_end = to.z;
    double roll_ref_end  = to.roll_ref;
    double pitch_ref_end = to.pitch_ref;
    double thrust_end    = to.thrust;

    /* Linear drag model: Verify the constant and the parameters! */
    IntermediateState dragacc1 = linear_drag_coefficient1*velocity1;
    IntermediateState dragacc2 = linear_drag_coefficient2*velocity2;


	// DIFFERENTIAL EQUATION:
    // ------------------------------
    f << dot(velocity1)      == ((cos(roll)*cos(yaw)*sin(pitch) + sin(roll)*sin(yaw))*thrust - dragacc1);
    f << dot(velocity2)      == ((cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll))*thrust - dragacc2);
    f << dot(velocity3)      == (-g + cos(pitch)*cos(roll)*thrust);
    f << dot(roll)           == (roll_gain*roll_ref - roll)/roll_tau;
    f << dot(pitch)          == (pitch_gain*pitch_ref - pitch)/pitch_tau;
    f << dot(yaw)            == 0;
    f << dot(position1)      == velocity1;
    f << dot(position2)      == velocity2;
    f << dot(position3)      == velocity3;
    f << dot(roll_ref)       == roll_ref_dot;
    f << dot(pitch_ref)      == pitch_ref_dot;
    f << dot(thrust)         == thrust_dot;
    f << dot(roll_ref_dot)   == roll_ref_ddot;
    f << dot(pitch_ref_dot)  == pitch_ref_ddot;
    f << dot(thrust_dot)     == thrust_ddot;
	
	// OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
    IntermediateState cost_rpp       = roll_ref*roll_ref + pitch_ref*pitch_ref; //1/100*(thrust-g)*(thrust-g);
    IntermediateState cost_rppt_dot  = roll_ref_dot*roll_ref_dot + pitch_ref_dot*pitch_ref_dot + thrust_dot * thrust_dot;
    IntermediateState cost_rppt_ddot = roll_ref_ddot*roll_ref_ddot + pitch_ref_ddot*pitch_ref_ddot + thrust_ddot * thrust_ddot;

    double time_smooth_ratio = 4;
    OCP ocp(0.0, T, nSegments);
    ocp.minimizeLagrangeTerm(1/time_smooth_ratio * (1*cost_rpp + 1*cost_rppt_dot + 1/2*cost_rppt_ddot));
    ocp.minimizeMayerTerm(T);
    ocp.subjectTo(f);

    // Initial conditions
    // ----------------------------------
    ocp.subjectTo(AT_START, velocity1 == velocity1_init);
    ocp.subjectTo(AT_START, velocity2 == velocity2_init);
    ocp.subjectTo(AT_START, velocity3 == velocity3_init);
    ocp.subjectTo(AT_START, roll      == roll_init);
    ocp.subjectTo(AT_START, pitch     == pitch_init);
    ocp.subjectTo(AT_START, yaw       == yaw_init);
    ocp.subjectTo(AT_START, position1 == position1_init);
    ocp.subjectTo(AT_START, position2 == position2_init);
    ocp.subjectTo(AT_START, position3 == position3_init);
    ocp.subjectTo(AT_START, roll_ref  == roll_ref_init);
    ocp.subjectTo(AT_START, pitch_ref == pitch_ref_init);
    ocp.subjectTo(AT_START, thrust    == thrust_init);
    ocp.subjectTo(AT_START, roll_ref_dot  == 0);
    ocp.subjectTo(AT_START, pitch_ref_dot == 0);
    ocp.subjectTo(AT_START, thrust_dot    == 0);

    // Final conditions
    // ----------------------------------
    ocp.subjectTo(AT_END, velocity1 == velocity1_end);
    ocp.subjectTo(AT_END, velocity2 == velocity2_end);
    ocp.subjectTo(AT_END, velocity3 == velocity3_end);
    ocp.subjectTo(AT_END, roll      == roll_end);
    ocp.subjectTo(AT_END, pitch     == pitch_end);
    ocp.subjectTo(AT_END, yaw       == yaw_end);
    ocp.subjectTo(AT_END, position1 == position1_end);
    ocp.subjectTo(AT_END, position2 == position2_end);
    ocp.subjectTo(AT_END, position3 == position3_end);
    ocp.subjectTo(AT_END, roll_ref  == roll_ref_end);
    ocp.subjectTo(AT_END, pitch_ref == pitch_ref_end);
    ocp.subjectTo(AT_END, thrust    == thrust_end);
    ocp.subjectTo(AT_END, roll_ref_dot  == 0);
    ocp.subjectTo(AT_END, pitch_ref_dot == 0);
    ocp.subjectTo(AT_END, thrust_dot    == 0);

    // Bounds, TODO: add more bounds, i.e., velocities?
    // ----------------------------------
    
    double minTime = finalTime*0.5;
    double maxTime = finalTime*1.5;
    ocp.subjectTo(-0.8*28*PI/180 <= roll_ref  <= 0.8*28*PI/180); //Leave 20 % for control
    ocp.subjectTo(-0.8*28*PI/180 <= pitch_ref <= 0.8*28*PI/180); //Leave 20 % for control
    ocp.subjectTo(     1.2*g/2   <= thrust    <= 20*g*0.8);      //Leave 20 % for control
    ocp.subjectTo(       minTime <= T         <= maxTime);
    
    // velocity constraints:
    ocp.subjectTo( -5 <= velocity1 <= 5); //vx
    ocp.subjectTo( -5 <= velocity2 <= 5); //vy
    ocp.subjectTo( -5 <= velocity3 <= 5); //vz
    
    
    
    //ocp.subjectTo(-0.8*10*PI/180 <= roll_ref  <= 0.8*10*PI/180); //Leave 20 % for control
    //ocp.subjectTo(-0.8*10*PI/180 <= pitch_ref <= 0.8*10*PI/180); //Leave 20 % for control
    //ocp.subjectTo(   5*1.2   <= thrust    <= 15*1.2);     //Leave 20 % for control
    
 
	// OPTIMIZATION ALGORITHM:
	// ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp);

    // Add initial guess:
    //-----------------------------------------------------
    Grid timeGrid(0.0, 1.0, 2);
    //VariablesGrid p_init(15, timeGrid);
    VariablesGrid T_init( 1, timeGrid );
    /*
    p_init(0,0)  = velocity1_init;
    p_init(0,1)  = velocity2_init;
    p_init(0,2)  = velocity3_init;
    p_init(0,3)  = roll_init;
    p_init(0,4)  = pitch_init;
    p_init(0,5)  = yaw_init;
    p_init(0,6)  = position1_init;
    p_init(0,7)  = position2_init;
    p_init(0,8)  = position3_init;
    p_init(0,9)  = roll_ref_init;
    p_init(0,10) = pitch_ref_init;
    p_init(0,11) = thrust_init;
    p_init(0,12) = 0;
    p_init(0,13) = 0;
    p_init(0,14) = 0;

    p_init(1,0)  = velocity1_end;
    p_init(1,1)  = velocity2_end;
    p_init(1,2)  = velocity3_end;
    p_init(1,3)  = roll_end;
    p_init(1,4)  = pitch_end;
    p_init(1,5)  = yaw_end;
    p_init(1,6)  = position1_end;
    p_init(1,7)  = position2_end;
    p_init(1,8)  = position3_end;
    p_init(1,9)  = roll_ref_end;
    p_init(1,10) = pitch_ref_end;
    p_init(1,11) = thrust_end;
    p_init(1,12) = 0;
    p_init(1,13) = 0;
    p_init(1,14) = 0;
    */
    T_init(0,0 ) =  finalTime;

    //algorithm.initializeDifferentialStates( p_init );
    algorithm.initializeParameters        ( T_init );
    //For debug
#if DEBUGGING_PLOT
    GnuplotWindow window;

    window.addSubplot( position1, position2, "" , "x" , "y"); 
    window.addSubplot( position2, position3, "" , "y" , "z"); 
    window.addSubplot( position1, position3, "" , "x" , "z");
    window.addSubplot( velocity1, "","t","vx" );
    window.addSubplot( velocity2, "","t","vy" );
    window.addSubplot( velocity3, "","t","vz" );
    window.addSubplot( roll,      "","t","roll");
    window.addSubplot( pitch,     "","t","pitch");
    window.addSubplot( thrust,    "","t","thrust");
    // <--- end

    algorithm.set( MAX_NUM_ITERATIONS, 400 );

    algorithm << window;
#endif
    // Solve the problem:
    //----------------------------------------------------
	returnValue retVal = algorithm.solve();
    clearAllStaticCounters();


#if SUPRESS_ACADO_PRINT
	std::cout.clear();
#endif

	// test if optimization was successful 
	if (retVal != ACADO::returnValueType::SUCCESSFUL_RETURN)
		return false;

	VariablesGrid states, parameters, controls;
	algorithm.getDifferentialStates(states);
	algorithm.getControls(controls);
  algorithm.getParameters(parameters);
  opt_cost = algorithm.getObjectiveValue(); // store optimal cost
  //std::cout << "Opt cost: " << opt_cost << std::endl; 
    double finalT = parameters[0](0, 0);
    if (createContiniousTrajectory(outputTrajectory, states, finalT, nSegments))
	{
		return true;
	}
	return false;
}
