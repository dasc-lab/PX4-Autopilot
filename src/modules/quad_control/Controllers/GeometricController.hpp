#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
// #include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace matrix;

class GeometricController
{

  public:
    GeometricController();

    void set_gains();
    void update_state_pos(vehicle_local_position_s pos);
    void update_state_attitude(vehicle_attitude_s att);
    void update_state_Omega(vehicle_angular_velocity_s ang_vel);
    void update_setpoint(vehicle_local_position_setpoint_s sp);
    void run(); 

    float get_thrust_cmd();
    Vector3f get_torque_cmd();

 private:

    //PARAMETERS
    float kx, kv, kR, kOmega;
    float m, g;
    SquareMatrix<float, 3> J;

    // STATE
    Vector3f x, v, Omega;
    Dcmf R;

    // SETPOINT
    Vector3f x_ref, v_ref, a_ref, b1_ref, Omega_ref, alpha_ref;
    float yaw_ref;

    // results
    float thrust_cmd;
    Vector3f torque_cmd;

};
