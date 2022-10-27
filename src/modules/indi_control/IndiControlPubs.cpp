
// Indi Controller publish functions
// Devansh Agrawal Oct 2022

#include "IndiControl.hpp"

void IndiControl::publish_thrust_cmd() {

  vehicle_thrust_setpoint_s msg;
  msg.timestamp = hrt_absolute_time();
  msg.timestamp_sample = _now;
  msg.xyz[0] = 0.0;
  msg.xyz[1] = 0.0;
  msg.xyz[2] =
      math::constrain(_thrust_cmd * 1.5f * _thrust_constant, -1.0f, 1.0f);

  if (_armed) {
    // PX4_INFO("THRUST: %f, MSG: %f, K: %f", (double)_thrust_cmd,
    // (double)msg.xyz[2], (double)_thrust_constant);
  }

  _vehicle_thrust_setpoint_pub.publish(msg);
}

void IndiControl::publish_xi_cmd() {
  // TODO
}

void IndiControl::publish_ang_accel_cmd() {
  // TODO
}

void IndiControl::publish_torque_cmd() {

  // vehicle_torque_setpoint_s msg{};
  // msg.timestamp = hrt_absolute_time();
  // msg.timestamp_sample = _now;

  // for (size_t i = 0; i < 3; i++) {
  //   msg.xyz[i] =  math::constrain(_torque_cmd(i) * 0.005f * _torque_constant,
  //   -1.0f, 1.0f);
  // }

  // _vehicle_torque_setpoint_pub.publish(msg);
}

void IndiControl::publish_pwm_cmd() {

  // directly publish the cmd to actuator_motors
  actuator_motors_s msg;
  msg.timestamp = hrt_absolute_time();
  msg.timestamp_sample = _now;

  msg.reversible_flags = 255; // _param_r_rev.get(); // TODO check!

  for (size_t i = 0; i < 4; i++) {
    msg.control[i] = _pwm_cmd(i);
  }

  _actuator_motors_pub.publish(msg);

  actuator_controls_s msg2;
  msg2.timestamp = msg.timestamp;
  msg2.timestamp_sample = _now;
  msg2.control[actuator_controls_s::INDEX_ROLL] = _pwm_cmd(0);
  msg2.control[actuator_controls_s::INDEX_PITCH] = _pwm_cmd(1);
  msg2.control[actuator_controls_s::INDEX_YAW] = _pwm_cmd(2);
  msg2.control[actuator_controls_s::INDEX_THROTTLE] = _pwm_cmd(3);

  _actuator_controls_pub.publish(msg2);
}

void IndiControl::publish_attitude_setpoint(const Dcm<float> rotDes) {

  vehicle_attitude_setpoint_s msg;
  msg.timestamp = hrt_absolute_time();
  Euler<float> euler(rotDes);

  msg.roll_body = euler.phi();
  msg.pitch_body = euler.theta();
  msg.yaw_body = euler.psi();

  Quatf q_d(rotDes);
  q_d.copyTo(msg.q_d);

  _vehicle_attitude_setpoint_pub.publish(msg);
}

void IndiControl::publish_rates_setpoint(const Vector3f ang_acc) {

  vehicle_rates_setpoint_s msg;
  msg.timestamp = hrt_absolute_time();
  msg.roll = ang_acc(0);
  msg.pitch = ang_acc(1);
  msg.yaw = ang_acc(2);

  _vehicle_rates_setpoint_pub.publish(msg);
}
