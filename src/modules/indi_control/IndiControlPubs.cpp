
// Indi Controller publish functions
// Devansh Agrawal Oct 2022


#include "IndiControl.hpp"

void IndiControl::publish_thrust_cmd()
{

  vehicle_thrust_setpoint_s msg;
  msg.timestamp = hrt_absolute_time();
  msg.timestamp_sample = _now;
  msg.xyz[0] = 0.0;
  msg.xyz[1] = 0.0;
  msg.xyz[2] = math::constrain(_thrust_cmd * 1.5f * _thrust_constant, -1.0f, 1.0f);

  if (_armed) {
    // PX4_INFO("THRUST: %f, MSG: %f, K: %f", (double)_thrust_cmd, (double)msg.xyz[2], (double)_thrust_constant);
  }

  _vehicle_thrust_setpoint_pub.publish(msg);
}

void IndiControl::publish_xi_cmd()
{
  // TODO
}

void IndiControl::publish_ang_accel_cmd()
{
  // TODO
}

void IndiControl::publish_torque_cmd()
{

  vehicle_torque_setpoint_s msg{};
  msg.timestamp = hrt_absolute_time();
  msg.timestamp_sample = _now;

  for (size_t i = 0; i < 3; i++) {
    msg.xyz[i] =  math::constrain(_torque_cmd(i) * 0.005f * _torque_constant, -1.0f, 1.0f);
  }

  _vehicle_torque_setpoint_pub.publish(msg);

}

