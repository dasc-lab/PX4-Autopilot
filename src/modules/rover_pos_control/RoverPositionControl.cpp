/****************************************************************************
 *
 *   Copyright (c) 2017, 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include "RoverPositionControl.hpp"
#include <lib/geo/geo.h>

#define ACTUATOR_PUBLISH_PERIOD_MS 4

using namespace matrix;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);

RoverPositionControl::RoverPositionControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED,  MODULE_NAME": cycle")) // TODO : do we even need these perf counters
{
}

RoverPositionControl::~RoverPositionControl()
{
	perf_free(_loop_perf);
}

bool
RoverPositionControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void RoverPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_gnd_control.set_l1_damping(_param_l1_damping.get());
		_gnd_control.set_l1_period(_param_l1_period.get());
		_gnd_control.set_l1_roll_limit(math::radians(0.0f));

		pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_speed_ctrl,
				   _param_speed_p.get(),
				   _param_speed_i.get(),
				   _param_speed_d.get(),
				   _param_speed_imax.get(),
				   _param_gndspeed_max.get());

	        pid_init(&_angular_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_angular_speed_ctrl,
				   _param_angular_speed_p.get(),
				   _param_angular_speed_i.get(),
				   _param_angular_speed_d.get(),
				   _param_angular_speed_imax.get(),
				   _param_gndspeed_max.get());
	}
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	if (_control_mode.flag_control_manual_enabled) {
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {
			float dt = math::constrain(hrt_elapsed_time(&_manual_setpoint_last_called) * 1e-6f,  0.0002f, 0.04f);

			if (!_control_mode.flag_control_climb_rate_enabled &&
			    !_control_mode.flag_control_offboard_enabled) {

				if (_control_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs
					_att_sp.roll_body = 0.0;
					_att_sp.pitch_body = 0.0;

					/* reset yaw setpoint to current position if needed */
					if (_reset_yaw_sp) {
						const float vehicle_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
						_manual_yaw_sp = vehicle_yaw;
						_reset_yaw_sp = false;

					} else {
						const float yaw_rate = math::radians(_param_gnd_man_y_max.get());
						_att_sp.yaw_sp_move_rate = _manual_control_setpoint.y * yaw_rate;
						_manual_yaw_sp = wrap_pi(_manual_yaw_sp + _att_sp.yaw_sp_move_rate * dt);
					}

					_att_sp.yaw_body = _manual_yaw_sp;
					_att_sp.thrust_body[0] = _manual_control_setpoint.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();


					_attitude_sp_pub.publish(_att_sp);

				} else {
					_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0.0f; // Nominally roll: _manual_control_setpoint.y;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0.0f; // Nominally pitch: -_manual_control_setpoint.x;
					// Set heading from the manual roll input channel
					_act_controls.control[actuator_controls_s::INDEX_YAW] =
						_manual_control_setpoint.y; // Nominally yaw: _manual_control_setpoint.r;
					// Set throttle from the manual throttle channel
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = _manual_control_setpoint.z;
					_reset_yaw_sp = true;
				}

			} else {
				_reset_yaw_sp = true;
			}

			_manual_setpoint_last_called = hrt_absolute_time();
		}
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
	}
}

void
RoverPositionControl::attitude_setpoint_poll()
{
	if (_att_sp_sub.updated()) {
		_att_sp_sub.copy(&_att_sp);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}

void
RoverPositionControl::control_local_position(const matrix::Vector2f &current_position,
				       const matrix::Vector3f &current_velocity, const matrix::Vector3f &current_angular_velocity)
{
	// TODO: validity check

	if ((_control_mode.flag_control_auto_enabled ||
	     _control_mode.flag_control_offboard_enabled) ) {

		/* AUTONOMOUS FLIGHT */
		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* current waypoint (the one currently heading for) */
		matrix::Vector2f curr_sp(_trajectory_setpoint.x, _trajectory_setpoint.y);
                matrix::Vector2f pos_error( current_position(0)-curr_sp(0), current_position(1)-curr_sp(1) );
		float dist_target = pos_error.norm();//sqrt( dist_sq );
		// PX4_WARN("Error pos: %f %f %f",(double)pos_error(0), (double)pos_error(1),(double)dist_target);

	        switch (_pos_ctrl_state) {
		case GOTO_WAYPOINT: {
				if (dist_target < 0.1f){//(double)_param_nav_loiter_rad.get()) {
					_pos_ctrl_state = STOPPING;  // We are closer than loiter radius to waypoint, stop.

				} else {
					Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

					// Yaw Error
					// float current_yaw = 2.0f * atan2f( _vehicle_att.q[3], _vehicle_att.q[0] );
					float current_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
					float desired_yaw = atan2f( -pos_error(1), -pos_error(0) );
					float yaw_error = desired_yaw - current_yaw;

					// Clip yaw to -pi to pi
					if (yaw_error > M_PI_F){
						yaw_error = 2.0f * M_PI_F - yaw_error;
					}
					else if (yaw_error < -M_PI_F){
						yaw_error = 2.0f * M_PI_F + yaw_error;
					}

					// Design angular velocity: Always run attitude control
					float yaw_rate_sp =  _param_rover_k_yaw.get() * yaw_error;
					float x_vel_sp = 0.0f;

					// Design linear velocity
					if ( (yaw_error>0.2f) || (yaw_error<-0.2f)){  // rotate only
						x_vel_sp = 0.0f;
					}
					else{					      // rotate and move
						x_vel_sp = cos(yaw_error) * _param_rover_k_pos.get() * pos_error.norm();
					}

					control_velocity(current_velocity, current_angular_velocity, x_vel_sp, yaw_rate_sp);
				}
			}
			break;

		case STOPPING: {
				_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				// PX4_WARN("Stopping- dist target: %f", (double)dist_target);
				if (dist_target > 0) {
					_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
				}
			}
			break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

	}

}


void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity, const matrix::Vector3f &current_angular_velocity, const float desired_linear_x_velocity, const float desired_angular_z_velocity)
{
	// assume desired velocity in body frame: Unicycle type of control

	float dt = 0.01; // Using non zero value to a avoid division by zero

	// get body frame current velocity
	const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
	const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));
	const float x_vel = vel(0);

	// get vehicle acceleration for derivative term
	// TODO: convert to body frame
	// const float x_acc = _vehicle_acceleration_sub.get().xyz[0];
	Vector3f acc_body = R_to_body * Vector3f( _vehicle_acceleration_sub.get().xyz[0], _vehicle_acceleration_sub.get().xyz[1], _vehicle_acceleration_sub.get().xyz[2] );
	const float x_acc = acc_body(0);

	// get Thrust command
	if (desired_linear_x_velocity>0.0){
		const float vel_ff = _param_linear_deadband_pos.get() * ( 1.0f - _param_rover_vel_ff.get() * desired_linear_x_velocity   ) + _param_rover_vel_ff.get() * desired_linear_x_velocity;
		const float control_throttle =  vel_ff +  pid_calculate(&_speed_ctrl, desired_linear_x_velocity, x_vel, x_acc, dt);
	// const float control_throttle =  _param_rover_vel_ff.get() * desired_linear_x_velocity +  pid_calculate(&_speed_ctrl, desired_linear_x_velocity, x_vel, x_acc, dt);
	}
	else{
		const float vel_ff = _param_linear_deadband_neg.get() * ( 1.0f - _param_rover_vel_ff.get() * desired_linear_x_velocity   ) + _param_rover_vel_ff.get() * desired_linear_x_velocity;
		const float control_throttle =  vel_ff +  pid_calculate(&_speed_ctrl, desired_linear_x_velocity, x_vel, x_acc, dt);
	}
	//Constrain maximum throttle to mission throttle
	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -_param_throttle_max.get(), _param_throttle_max.get());

	// get angular torque command
	const float yaw_ff = _param_angular_deadband.get() * ( 1.0f - _param_rover_omg_ff.get() * desired_angular_z_velocity   ) + _param_rover_omg_ff.get() * desired_angular_z_velocity;
	const float control_yaw = yaw_ff + pid_calculate(&_angular_speed_ctrl, desired_angular_z_velocity, current_angular_velocity(2), 0.0, dt);
	_act_controls.control[actuator_controls_s::INDEX_YAW] = math::constrain(control_yaw, -1.0f, 1.0f);
}

void
RoverPositionControl::control_attitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp)
{
	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = Quatf(att.q).inversed() * Quatf(att_sp.q_d);
	const Eulerf euler_sp = qe;

	float control_effort = euler_sp(2) / _param_max_turn_angle.get();
	control_effort = math::constrain(control_effort, -1.0f, 1.0f);

	_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	const float control_throttle = att_sp.thrust_body[0];

	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =  math::constrain(control_throttle, 0.0f, 1.0f);

}

void
RoverPositionControl::Run()
{
	parameters_update(true);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// PX4_WARN("Inside rover!!");
		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		attitude_setpoint_poll();
		vehicle_attitude_poll();
		manual_control_setpoint_poll();

		_vehicle_acceleration_sub.update();

		/* update parameters from storage */
		parameters_update();

		/* only run controller if position changed */
		if (_local_pos_sub.update(&_local_pos)) {

			// position global but velocity local?? ///////////////////////////////////////////////////////////////
			matrix::Vector3f ground_speed(_local_pos.vx, _local_pos.vy,  _local_pos.vz);
			// matrix::Vector2d current_position(_global_pos.lat, _global_pos.lon);
			matrix::Vector2f current_position(_local_pos.x, _local_pos.y);
			matrix::Vector3f current_velocity(_local_pos.vx, _local_pos.vy, _local_pos.vz);
			matrix::Vector3f current_angular_velocity(angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]);

			//Position Control Mode
			if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {
				_trajectory_setpoint_sub.update(&_trajectory_setpoint);
				// PX4_WARN("%f %f %f",(double)_trajectory_setpoint.x, (double)_trajectory_setpoint.y, (double)_trajectory_setpoint.z);
				control_local_position(current_position, current_velocity, current_angular_velocity);
				// PX4_WARN("")

			// Velocity Control Mode
			} else if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_velocity_enabled) {
				_trajectory_setpoint_sub.update(&_trajectory_setpoint);
				// control_velocity(current_velocity, current_angular_velocity);
				Vector3f desired_velocity{_trajectory_setpoint.vx, _trajectory_setpoint.vy, _trajectory_setpoint.vz};
				float desired_linear_x_velocity = desired_velocity(0);
				float desired_angular_z_velocity = _trajectory_setpoint.yawspeed;
				control_velocity(current_velocity, current_angular_velocity, desired_linear_x_velocity, desired_angular_z_velocity);
			}
		}

		// Respond to an attitude update and run the attitude controller if enabled
		if (_control_mode.flag_control_attitude_enabled
		    && !_control_mode.flag_control_position_enabled
		    && !_control_mode.flag_control_velocity_enabled) {
			control_attitude(_vehicle_att, _att_sp);
			// PX4_WARN("Attitude Mode");
		}

		/* Only publish if any of the proper modes are enabled */
		if (_control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_attitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_manual_enabled) {
			// timestamp and publish controls
			_act_controls.timestamp = hrt_absolute_time();
			_actuator_controls_pub.publish(_act_controls);

			vehicle_thrust_setpoint_s v_thrust_sp{};
			v_thrust_sp.timestamp = hrt_absolute_time();
			v_thrust_sp.xyz[0] = _act_controls.control[actuator_controls_s::INDEX_THROTTLE];
			v_thrust_sp.xyz[1] = 0.0f;
			v_thrust_sp.xyz[2] = 0.0f;
			_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

			vehicle_torque_setpoint_s v_torque_sp{};
			v_torque_sp.timestamp = hrt_absolute_time();
			v_torque_sp.xyz[0] = _act_controls.control[actuator_controls_s::INDEX_ROLL];
			v_torque_sp.xyz[1] = _act_controls.control[actuator_controls_s::INDEX_PITCH];
			v_torque_sp.xyz[2] = _act_controls.control[actuator_controls_s::INDEX_YAW];
			_vehicle_torque_setpoint_pub.publish(v_torque_sp);
			// PX4_WARN("PUBLISHING.... %f %f",(double)v_thrust_sp.xyz[0], (double)v_torque_sp.xyz[2]);
		}
	}
}

int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	RoverPositionControl *instance = new RoverPositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at IMU_GYRO_RATEMAX.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_pos_control_main(int argc, char *argv[])
{
	return RoverPositionControl::main(argc, argv);
}
