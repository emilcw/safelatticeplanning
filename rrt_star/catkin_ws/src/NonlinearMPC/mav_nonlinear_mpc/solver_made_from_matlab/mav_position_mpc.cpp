/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState velocity1;
    DifferentialState velocity2;
    DifferentialState velocity3;
    DifferentialState roll;
    DifferentialState pitch;
    DifferentialState yaw;
    DifferentialState position1;
    DifferentialState position2;
    DifferentialState position3;
    Control roll_ref;
    Control pitch_ref;
    Control thrust;
    OnlineData roll_tau; 
    OnlineData roll_gain; 
    OnlineData pitch_tau; 
    OnlineData pitch_gain; 
    OnlineData linear_drag_coefficient1; 
    OnlineData linear_drag_coefficient2; 
    OnlineData external_forces1; 
    OnlineData external_forces2; 
    OnlineData external_forces3; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "mav_position_mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mav_position_mpc_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << position1;
    acadodata_f1 << position2;
    acadodata_f1 << position3;
    acadodata_f1 << velocity1;
    acadodata_f1 << velocity2;
    acadodata_f1 << velocity3;
    acadodata_f1 << roll;
    acadodata_f1 << pitch;
    acadodata_f1 << roll_ref;
    acadodata_f1 << pitch_ref;
    acadodata_f1 << (-9.80659999999999953957e+00+cos(pitch)*cos(roll)*thrust);
    Function acadodata_f2;
    acadodata_f2 << position1;
    acadodata_f2 << position2;
    acadodata_f2 << position3;
    acadodata_f2 << velocity1;
    acadodata_f2 << velocity2;
    acadodata_f2 << velocity3;
    OCP ocp1(0, 2, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo((-7.85398163397448278999e-01) <= roll_ref <= 7.85398163397448278999e-01);
    ocp1.subjectTo((-7.85398163397448278999e-01) <= pitch_ref <= 7.85398163397448278999e-01);
    ocp1.subjectTo(4.90329999999999976978e+00 <= thrust <= 1.47098999999999993094e+01);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(velocity1) == (-(-sin(pitch))*linear_drag_coefficient1*thrust*velocity3+(cos(roll)*cos(yaw)*sin(pitch)+sin(roll)*sin(yaw))*thrust-cos(pitch)*cos(yaw)*linear_drag_coefficient1*thrust*velocity1-cos(pitch)*linear_drag_coefficient1*sin(yaw)*thrust*velocity2+external_forces1);
    acadodata_f3 << dot(velocity2) == (-(-cos(roll)*sin(yaw)+cos(yaw)*sin(pitch)*sin(roll))*linear_drag_coefficient2*thrust*velocity1-(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw))*linear_drag_coefficient2*thrust*velocity2+(cos(roll)*sin(pitch)*sin(yaw)-cos(yaw)*sin(roll))*thrust-cos(pitch)*linear_drag_coefficient2*sin(roll)*thrust*velocity3+external_forces2);
    acadodata_f3 << dot(velocity3) == (-9.80659999999999953957e+00+cos(pitch)*cos(roll)*thrust+external_forces3);
    acadodata_f3 << dot(roll) == (-roll+roll_gain*roll_ref)/roll_tau;
    acadodata_f3 << dot(pitch) == (-pitch+pitch_gain*pitch_ref)/pitch_tau;
    acadodata_f3 << dot(yaw) == 0.00000000000000000000e+00;
    acadodata_f3 << dot(position1) == velocity1;
    acadodata_f3 << dot(position2) == velocity2;
    acadodata_f3 << dot(position3) == velocity3;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 3 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 9 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LINEAR_ALGEBRA_SOLVER");
    options_flag = ExportModule1.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: IMPLICIT_INTEGRATOR_NUM_ITS");
    options_flag = ExportModule1.set( CG_USE_OPENMP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_USE_OPENMP");
    options_flag = ExportModule1.set( CG_HARDCODE_CONSTRAINT_VALUES, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_HARDCODE_CONSTRAINT_VALUES");
    options_flag = ExportModule1.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_USE_VARIABLE_WEIGHTING_MATRIX");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "." );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

