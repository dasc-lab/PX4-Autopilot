
/**
 * @file IndiPositionControl.cpp
 */

#include "IndiPositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

void IndiPositionControl::setState(const IndiPositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void IndiPositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool IndiPositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (!valid){
		PX4_WARN("INPUT IS NOT VALID");
		return valid;
	}

	Vector3f pos_error = _pos - _pos_sp;
	Vector3f vel_error = _vel - _vel_sp;

	_acc_sp = _acc_sp - _kp * pos_error - _kv * vel_error;

	_thr_sp = (_mass * _acc_sp + Vector3f(0,0, - 9.81f)); // in inertial frame
	
  // TODO: yaw control?

	// PX4_INFO("pos control thrust sp: %f", (double)_thr_sp(2));

	return valid;
}

// void IndiPositionControl::_positionControl()
// {
// 	// P-position controller
// 	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
// 	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
// 	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
// 	// make sure there are no NAN elements for further reference while constraining
// 	ControlMath::setZeroIfNanVector3f(vel_sp_position);
// 	PX4_INFO("_vel_sp: %f", (double)(_vel_sp.norm()));
// 
// 	// Constrain horizontal velocity by prioritizing the velocity component along the
// 	// the desired position setpoint over the feed-forward term.
// 	//_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
// 	// Constrain velocity in z-direction.
// 	//_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
// }
// 
// void IndiPositionControl::_velocityControl(const float dt)
// {
// 	// PID velocity control
// 	Vector3f vel_error = _vel_sp - _vel;
// 	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
// 
// 	// No control input from setpoints or corresponding states which are NAN
// 	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
// 	PX4_INFO("_acc_sp: %f", (double)(_acc_sp.norm()));
// 
// 	_accelerationControl();
// 
// 	// Integrator anti-windup in vertical direction
// 	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
// 	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
// 		vel_error(2) = 0.f;
// 	}
// 
// 	// Prioritize vertical control while keeping a horizontal margin
// 	const Vector2f thrust_sp_xy(_thr_sp);
// 	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
// 	const float thrust_max_squared = math::sq(_lim_thr_max);
// 
// 	// Determine how much vertical thrust is left keeping horizontal margin
// 	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
// 	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);
// 
// 	// Saturate maximal vertical thrust
// 	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));
// 
// 	// Determine how much horizontal thrust is left after prioritizing vertical control
// 	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
// 	float thrust_max_xy = 0;
// 
// 	if (thrust_max_xy_squared > 0) {
// 		thrust_max_xy = sqrtf(thrust_max_xy_squared);
// 	}
// 
// 	// Saturate thrust in horizontal direction
// 	if (thrust_sp_xy_norm > thrust_max_xy) {
// 		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
// 	}
// 
// 	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
// 	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
// 	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
// 	const float arw_gain = 2.f / _gain_vel_p(0);
// 	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));
// 
// 	// Make sure integral doesn't get NAN
// 	ControlMath::setZeroIfNanVector3f(vel_error);
// 	// Update integral part of velocity control
// 	_vel_int += vel_error.emult(_gain_vel_i) * dt;
// 
// 	// limit thrust integral
// 	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
// }
// 
// void IndiPositionControl::_accelerationControl()
// {
// 	// Assume standard acceleration due to gravity in vertical direction for attitude generation
// 	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
// 	PX4_INFO("acc_control body_z: %f, %f, %f", (double)body_z(0), (double)body_z(1), (double)body_z(2));
// 	//ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
// 	// Scale thrust assuming hover thrust produces standard gravity
// 	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
// 	PX4_INFO("hover_thrust: %f", (double)_hover_thrust);
// 	// Project thrust to planned body attitude
// 	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
// 	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
// 	_thr_sp = body_z * collective_thrust;
// }

bool IndiPositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void IndiPositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void IndiPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
