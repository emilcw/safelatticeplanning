#include "State.h"


State::State()
{
    // Position
    this->x = 0;
    this->y = 0;
    this->z = 0;
    // Velocity
    this->vx= 0;
    this->vy= 0;
    this->vz= 0;
    // Orientation
    this->roll  = 0;
    this->pitch = 0;
    this->yaw   = 0;
    // Controls
    this->roll_ref  = 0;
    this->pitch_ref = 0;
    this->thrust    = 9.806;
    // help variables
    this->roll_ref_dot  = 0;
    this->pitch_ref_dot = 0;
    this->thrust_dot    = 0;
    this->time = 0;

}

State::State(double x_in, double y_in, double z_in, double vx_in, double vy_in, double vz_in,
             double roll_in, double pitch_in, double yaw_in, double roll_ref_in, double pitch_ref_in,
             double thrust_in, double roll_ref_dot_in, double pitch_ref_dot_in, double thrust_dot_in, double time_in)
  : x(x_in), y(y_in), z(z_in), vx(vx_in), vy(vy_in), vz(vz_in), roll(roll_in), pitch(pitch_in), 
  yaw(yaw_in), roll_ref(roll_ref_in), thrust(thrust_in) {

    this->roll_ref_dot  = roll_ref_dot_in;
    this->pitch_ref_dot = pitch_ref_dot_in;
    this->thrust_dot    = thrust_dot_in;
    this->time = time_in;
}

State::State(const State& stateIn)
{
    this->x         = stateIn.x;
    this->y         = stateIn.y;
    this->z         = stateIn.z;
    this->vx        = stateIn.vx;
    this->vy        = stateIn.vy;
    this->vz        = stateIn.vz;
    this->roll      = stateIn.roll;
    this->pitch     = stateIn.pitch;
    this->yaw       = stateIn.yaw;
    // Controls
    this->roll_ref  = stateIn.roll_ref;
    this->pitch_ref = stateIn.pitch_ref;
    this->thrust    = stateIn.thrust;
    // help variables
    this->roll_ref_dot  = stateIn.roll_ref_dot;
    this->pitch_ref_dot = stateIn.pitch_ref_dot;
    this->thrust_dot    = stateIn.thrust_dot;
    this->time          = stateIn.time;
}

void State::operator=(const State&  other)
{
    this->x         = other.x;
    this->y         = other.y;
    this->z         = other.z;
    this->vx        = other.vx;
    this->vy        = other.vy;
    this->vz        = other.vz;
    this->roll      = other.roll;
    this->pitch     = other.pitch;
    this->yaw       = other.yaw;
    // Controls
    this->roll_ref  = other.roll_ref;
    this->pitch_ref = other.pitch_ref;
    this->thrust    = other.thrust;
    // help variables
    this->roll_ref_dot  = other.roll_ref_dot;
    this->pitch_ref_dot = other.pitch_ref_dot;
    this->thrust_dot    = other.thrust_dot;
    this->time          = other.time;
}

void State::print()
{
    std::cout << "x: " << x << " y: " << y << " z: " << z << " vx: " << vx << " vy: " << vy  <<
                 " vz: " << vz << " r: " << roll << " p: " << pitch << " yaw: " << yaw << " thrust: " << thrust << std::endl;
}

