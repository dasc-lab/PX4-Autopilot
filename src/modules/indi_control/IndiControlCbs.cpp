// Indi Controller Callbacks
// Devansh Agrawal Oct 2022

#include "IndiControl.hpp"
#include <lib/diffflat/DiffFlatQuad.hpp>

void IndiControl::cb_params() {
  // Check if parameters have changed
  if (_parameter_update_sub.updated()) {
    // clear update
    parameter_update_s param_update;
    _parameter_update_sub.copy(&param_update);
    updateParams(); // update module parameters (in DEFINE_PARAMETERS)
    update_parameters();
  }
}

void IndiControl::cb_vehicle_status() {
  if (_vehicle_status_sub.updated()) {

    if (_vehicle_status_sub.copy(&_vehicle_status)) {

      const bool armed = (_vehicle_status.arming_state ==
                          vehicle_status_s::ARMING_STATE_ARMED);

      if (armed && !_armed) {
        PX4_WARN("vehicle armed due to %d",
                 _vehicle_status.latest_arming_reason);
        _start = hrt_absolute_time();

      } else if (!armed && _armed) {
        PX4_INFO("vehicle disarmed due to %d",
                 _vehicle_status.latest_disarming_reason);
      }

      _armed = armed;
    }
  }
}

void IndiControl::cb_vehicle_control_mode() {

  if (_vehicle_control_mode_sub.updated()) {
    _vehicle_control_mode_sub.copy(&_vehicle_control_mode);
  }
}

void IndiControl::cb_manual_control_setpoint() {

  if (_manual_control_setpoint_sub.updated()) {
    _manual_control_setpoint_sub.copy(&_manual_control_setpoint);
  }
}

void IndiControl::cb_vehicle_local_position() {
  if (_vehicle_local_position_sub.updated()) {
    _vehicle_local_position_sub.copy(&_local_position);
    // const float g = 9.81;
    // Vector3f a ( _local_position.ax, _local_position.ay, _local_position.az +
    // g); _a_filt = _a_filter.apply(a);

    _vehicle_local_position_ready = true;
  }
}

void IndiControl::cb_vehicle_attitude() {
  if (_vehicle_attitude_sub.updated()) {
    _vehicle_attitude_sub.copy(&_attitude);
    Quatf q(_attitude.q);
    Vector3f ez(0, 0, 1);
    _bz = q.rotateVector(ez);
    _vehicle_attitude_ready = true;
  }
}

void IndiControl::cb_sensor_accel() {
  if (_sensor_accel_sub.updated()) {
    sensor_accel_s accel;
    _sensor_accel_sub.copy(&accel);
    Vector3f a_body(accel.x, accel.y, accel.z);
    Quatf q(_attitude.q);
    _a = q.rotateVector(a_body) + Vector3f(0, 0, 9.81);
    _sensor_accel_ready = true;
  }
}

void IndiControl::cb_vehicle_angular_velocity() {
  if (_vehicle_angular_velocity_sub.updated()) {
    _vehicle_angular_velocity_sub.copy(&_angular_velocity);
    for (size_t i = 0; i < 3; i++) {
      _ang_vel(i) = _angular_velocity.xyz[i];
    }
    if (!_ang_vel.has_nan()) {
      _ang_vel_filt = _ang_vel_filter.apply(_ang_vel);
    }
    _ang_vel_ready = true;
  }
}

void IndiControl::cb_vehicle_angular_acceleration() {
  if (_vehicle_angular_acceleration_sub.updated()) {
    vehicle_angular_acceleration_s msg;
    _vehicle_angular_acceleration_sub.copy(&msg);
    for (size_t i = 0; i < 3; i++) {
      _ang_accel(i) = msg.xyz[i];
    }
    if (!_ang_accel.has_nan()) {
      _ang_accel_filt = _ang_accel_filter.apply(_ang_accel);
    }
    _ang_acc_ready = true;
  }
}

void IndiControl::cb_diffflat_setpoint() {
  if (_diffflat_setpoint_sub.updated()) {
     _diffflat_setpoint_sub.copy(&_flat_setpoint);
  }

  _flat_setpoint_ready = true;

  
  // set default setpoint
  for (size_t i=0; i<3; i++){
	  _flat_setpoint.pos[i] = 0.0f;
	  _flat_setpoint.vel[i] = 0.0f;
	  _flat_setpoint.acc[i] = 0.0f;
	  _flat_setpoint.jerk[i] = 0.0f;
	  _flat_setpoint.snap[i] = 0.0f;
  }
  _flat_setpoint.yaw = 0.0; 
  _flat_setpoint.yaw_rate = 0.0f;
  _flat_setpoint.yaw_accel = 0.0f;

  _flat_setpoint.pos[2] = 1.25; // in ENU

  // convert to a trackable setpoint
  diffflat::flat_state_ENU_to_quad_state_FRD(
		  _pos_sp, 
		  _vel_sp,
		  _acc_sp,
		  _b1d_sp,
		  _ang_vel_sp,
		  _ang_acc_sp,
		  Vector3f(_flat_setpoint.pos),
		  Vector3f(_flat_setpoint.vel),
		  Vector3f(_flat_setpoint.acc),
		  Vector3f(_flat_setpoint.jerk),
		  Vector3f(_flat_setpoint.snap),
		  _flat_setpoint.yaw,
		  _flat_setpoint.yaw_rate,
		  _flat_setpoint.yaw_accel
  );


}

