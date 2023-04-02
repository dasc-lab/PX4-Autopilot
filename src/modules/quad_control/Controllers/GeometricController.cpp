
// Devansh Agrawal 
// April 2023

#include "GeometricController.hpp"

using namespace matrix;


GeometricController::GeometricController(){

  set_gains();
  
}


void GeometricController::set_gains(){

  kx = 1.0f;
  kv = 2.0f;
  kR = 0.35f;
  kOmega = 0.15f;

  m = 0.8f;
  g = 9.81f;
  J.setZero();
  J(0,0) = 0.05;
  J(1,1) = 0.05;
  J(2,2) = 0.09;
  
}


void GeometricController::update_state_pos(vehicle_local_position_s pos)
{
  // grab pos and vel
  x(0) = pos.x;
  x(1) = pos.y;
  x(2) = pos.z;

  v(0) = pos.vx;
  v(1) = pos.vy;
  v(2) = pos.vz;
}


void GeometricController::update_state_attitude(vehicle_attitude_s att)
{
  // grab rotation from body frame to earth frame
  R = Dcmf(Quatf(att.q));
}

void GeometricController::update_state_Omega(vehicle_angular_velocity_s ang_vel)
{
  for (size_t i=0; i<3; i++){
    Omega(i) = ang_vel.xyz[i];
  }
}

void GeometricController::update_setpoint(vehicle_local_position_setpoint_s sp) {


  x_ref(0) = sp.x;
  x_ref(1) = sp.y;
  x_ref(2) = sp.z;

  v_ref(0) = sp.vx;
  v_ref(1) = sp.vy;
  v_ref(2) = sp.vz;

  for (size_t i=0; i<3; i++){
    a_ref(i) = sp.acceleration[i];
  }

  //TODO(dev): update
  yaw_ref = 0.0; // desired yaw
  b1_ref(0) = std::cosf(yaw_ref);
  b1_ref(1) = std::sinf(yaw_ref);
  b1_ref(2) = 0.0;
  
  //TODO(dev): update
  Omega_ref.setZero();
  alpha_ref.setZero();

}


// Run the controller
// control input is stored in omega
void GeometricController::run(){

  // assuming FRD frame for everything 

  const Vector3f e3 (0,0,1);

  Vector3f ex = x - x_ref;
  Vector3f ev = v - v_ref;

  Vector3f thrust = -kx * ex - kv * ev - m * g  * e3 + m * a_ref;

  Vector3f b3d = -thrust.unit();
  Vector3f b2d = (b3d.cross(b1_ref)).unit();
  Vector3f b1d = (b2d.cross(b3d)).unit();

  // construct desired rotation matrix
  Dcmf Rd;
  for (size_t i=0; i < 3 ; i++){
    Rd(i, 0) = b1d(i);
    Rd(i, 1) = b2d(i);
    Rd(i, 2) = b3d(i);
  }

  Vector3f eR = 0.5f * (Dcmf(Rd.T() * R - R.T() * Rd)).vee();
  Vector3f eOmega = Omega - R.T() * Rd * Omega_ref;

  thrust_cmd = -thrust.dot(R * e3);

  torque_cmd = -kR * eR - kOmega * eOmega + Omega.cross(J * Omega) - J * (Omega.hat() * R.T() * Rd * Omega_ref - R.T() * Rd * alpha_ref);

  return;

}

float GeometricController::get_thrust_cmd()
{
  return thrust_cmd;
}

Vector3f GeometricController::get_torque_cmd()
{
  return torque_cmd;
}



