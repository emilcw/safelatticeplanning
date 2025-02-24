// DJI Matrice 100 motion model implementation. 
// Based on model identified here https://github.com/ethz-asl/mav_dji_ros_interface/blob/master/dji_interface/cfg/raven/nonlinear_mpc_50hz.yaml  
// @author Olov Andersson <olov.a.andersson@liu.se>

#pragma once
//#include "motion_model.h"

#include <functional>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

#include "rk4.hpp"

// Motion model for the M100
namespace mm100 { 

const int NX = 9;
const int NU = 3;

#define TYPE float

using VectorN = Eigen::Matrix<TYPE, NX+NU, 1>;
using VectorNX = Eigen::Matrix<TYPE, NX, 1>;
using VectorNU = Eigen::Matrix<TYPE, NU, 1>;


enum state_index {VX, VY, VZ, ROLL, PITCH, YAW, PX, PY, PZ};
enum control_index {RREF, PREF, THR};

// Status: Currently an okay match against DJI hardware sim.
// TODO: 
// -Yaw control. Also, investigate yaw angle init.
// -Investigate drag constant vs. paper/linear mpc, can this account for thrust issues also? 
// -Investigate thrust issues, do they manually convert from m/s^2?

// Dynamics Differential Equation. NOTE: function of X and U.
inline VectorN dynamics_m100(float const t, const VectorN &xu) {
    auto g = Eigen::Vector3f(0,0,9.8066);

    // Parameters from: https://github.com/ethz-asl/mav_dji_ros_interface/blob/master/dji_interface/cfg/raven/nonlinear_mpc_50hz.yaml
    double roll_gain = 1.101;
    double pitch_gain = 1.097;
    double linear_drag_coefficient1 = 0.010000;
    double linear_drag_coefficient2 = 0.010000;
    double roll_tau = 0.253;
    double pitch_tau = 0.267;

    // Translation from MATLAB ACADO to Eigen, see: https://github.com/ethz-asl/mav_control_rw/tree/master/mav_nonlinear_mpc
    double roll = xu(ROLL);
    double pitch = xu(PITCH);
    double yaw = xu(YAW);
    double roll_ref = xu(NX+RREF);
    double pitch_ref = xu(NX+PREF);
    double thrust = xu(NX+THR)/100.0*2.0*g(2); // ROS DJI API input is 0-100(%). Guestimate from hardware sim is that acceleration minus drag at max thrust is ~2g but hover is ~36%??.
    auto velocity = xu.segment(VX, 3);

    Eigen::Matrix3f R;
    R << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)),
        cos(pitch)*sin(yaw), cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll), (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)),
        -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll);

    auto z_B = R.col(2);

    // Non-linear drag model
    Eigen::Matrix3f drag_mat = Eigen::Matrix3f::Zero(3,3);
    drag_mat(0,0) = linear_drag_coefficient1;
    drag_mat(1,1) = linear_drag_coefficient2;
//    auto drag_acc = thrust*drag_mat*R.transpose()*velocity; // TODO: This gets the wrong sign, re-read paper. Drag also seems much larger in DJI sim...
    auto drag_acc = drag_mat*velocity; // Drag seems larger in DJI sim...

    double droll = (1/roll_tau)*(roll_gain*roll_ref - roll);
    double dpitch = (1/pitch_tau)*(pitch_gain*pitch_ref - pitch);

    Eigen::Vector3f external_forces(0,0,0); // TODO: Disturbances...
    VectorN dot_f;
    dot_f << z_B*thrust-g-drag_acc+external_forces,
    droll,
    dpitch,
    0,
    velocity,
    0,0,0; // Added controls, zero-order-hold (piece-wise constant)
    //std::cerr << "xu=[" << xu.transpose() << "]. drag_acc=" << drag_acc.transpose() << " thr=" << thrust << std::endl;
    //std::cerr << "dot_f=[" << dot_f.transpose() << "]." << std::endl;
    return dot_f;
}

class MotionM100 {
private:
    double t = 0.0;
    int iter = 0;
    double ts;
    VectorNX x; 

    RK4<NX+NU,TYPE> *integrator;
  
 public:
    // Constructor: Start state and time step
    MotionM100(const VectorNX &x_init, double timestep) : x(x_init), ts(timestep) {    
        // First-order vector DE defining the M100 motion dynamics on augmented state-action vector
        std::function<VectorN(float const, const VectorN &)> dot_f = dynamics_m100;
	x(YAW) = M_PI/2; // TODO: Why is hardware sim rotated like this?
        integrator = new RK4<NX+NU,TYPE>(dot_f);
    }

    ~MotionM100() {    
        delete integrator;
    }

    double get_yaw() { return x(YAW); }
    void set_yaw(double yaw) { x(YAW) = yaw; }

    // Tick: Controls u, fixed altitude (2m)?
    VectorNX tick(VectorNU &u, bool fixed_alt = false, double initial_altitude = 0.5) {
        VectorN xu;
        xu << x, u;
        VectorN next_xu = integrator->integrate(t, xu, ts);
        x = next_xu.head(NX); 

	// Bounds checking
        if (fixed_alt) 
        {
            x(PZ) = 2.0;
            x(VZ) = 0.0;
        } 
        else if (x(PZ) < initial_altitude) //Force to the initial altitude before takeoff
        {
            x(PZ) = initial_altitude;
	        x(VZ) = 0.0;  

        } 
        else if (x(PZ) > 100.0) //Alititude maximum limit
        {
            x(PZ) = 100.0;
            x(VZ) = 0.0;
        }

	double ANGLE_MAX = 0.611; // 35 deg
	if (x(PITCH) > ANGLE_MAX) x(PITCH) = ANGLE_MAX;
	if (x(PITCH) < -ANGLE_MAX) x(PITCH) = -ANGLE_MAX;
	if (x(ROLL) > ANGLE_MAX) x(ROLL) = ANGLE_MAX;
	if (x(ROLL) < -ANGLE_MAX) x(ROLL) = -ANGLE_MAX;

	if (iter % (10) == 0) {
	    //std::cerr << "i=" << iter << ", x_t=[" << x.transpose() << "], u_t=[" << u.transpose() << "]." << std::endl;
	}
	iter++;
        return x;
    }

    void reset(const VectorNX &new_x = VectorNX::Zero(NX)) {
        x = new_x;        
        t = 0.0;
    }
};

}
