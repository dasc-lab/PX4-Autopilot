/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

using Vector4f = Vector<float, 4>;

MulticopterRateControl::MulticopterRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();

	//moving_error(0) = 0.0;
	//moving_error(1) = 0.0;
	//moving_error(2) = 0.0;
	

	construct_G_matrix();

	// construct J matrix
  _J.setZero();
	_J(0,0) = 0.005;
	_J(1,1) = 0.009;
	_J(2,2) = 0.009;
	_invJ = _J.I();
	
	// set mass
	_mass = 0.8;

}

void MulticopterRateControl::construct_G_matrix()
{
  
	// construct the G, Ginv matrix,
	// assumes the quadrotor is arranged as follows:
	//     +x
	//      ^
	//      |
	//      |
	//   2     0
	//      x       --> +y
	//   1     3
	//
	// where 1 is spinning ccw
	//
	// Use an FRD frame
	//
	// assume relationship is 
	// [thrust, torque] = G * omega.^2
	// 
	// where 
	//   thrust is in N
	//   torque is in Nm
	//   omega is in kilo-rad/s

  float L = 0.33 / 2.0; // arm_length 
  float LX = L / sqrt(2.0f);
  float LY = L / sqrt(2.0f);
	if (_jMAVSIM){
		_k_thrust = 4.0;
  	_k_torque = 0.05;
	}
	else {
		_k_thrust = 4.0;
		_k_torque = 0.05;
	}

	Vector3f rotor_pos_0 ( LX,  LY, 0);
	Vector3f rotor_pos_1 (-LX, -LY, 0);
	Vector3f rotor_pos_2 ( LX, -LY, 0);
	Vector3f rotor_pos_3 (-LX,  LY, 0);

	Vector3f iz (0,0,1);

	for (size_t motor_ind = 0; motor_ind < 4; motor_ind ++){
		_G(0, motor_ind) = -_k_thrust;
	}

	// TODO: check sign!
	Vector3f torque_0 = -_k_thrust * rotor_pos_0.cross(iz) + Vector3f(0,0,_k_torque);
	Vector3f torque_1 = -_k_thrust * rotor_pos_1.cross(iz) + Vector3f(0,0,_k_torque);
	Vector3f torque_2 = -_k_thrust * rotor_pos_2.cross(iz) - Vector3f(0,0,_k_torque);
	Vector3f torque_3 = -_k_thrust * rotor_pos_3.cross(iz) - Vector3f(0,0,_k_torque);

	for (size_t i=0; i<3; i++){
		_G(i+1, 0) = torque_0(i);
		_G(i+1, 1) = torque_1(i);
		_G(i+1, 2) = torque_2(i);
		_G(i+1, 3) = torque_3(i);
	}


	_invG = _G.I();

}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{

}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}
	
	_vehicle_status_sub.update(&_vehicle_status);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (armed && _vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// GRAB VEHICLE_ANGULAR_ACCEL (immediately)
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);
		const hrt_abstime now = angular_velocity.timestamp_sample;


		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		//const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};


		// GRAB rates setpoint
		vehicle_rates_setpoint_s v_rates_sp;
		if (_v_rates_sp_sub.update(&v_rates_sp)) {
			_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
			_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
			_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
			_thrust_sp = -v_rates_sp.thrust_body[2];
		}

		//_thrust_sp = 20.0f; // N
		
		// Run the rate controller
		
		float _kp = 1.0;
		float _kd = 1.7;
		Vector3f torque_cmd = _rates_sp + _kp * (_rates_sp - rates) - _kd * (angular_accel);

		//Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, false);
		//moving_error = 0.99f * moving_error + 0.01f * (rates - _rates_sp);

		// Do the mixing
		PX4_INFO("THRUST_SP: %f", (double)_thrust_sp);
		Vector4f omega = mix(torque_cmd, _thrust_sp);

		publish_actuator_outputs(omega);
	
	}

	perf_end(_loop_perf);
}


Vector4f MulticopterRateControl::mix(Vector3f torque_cmd, float thrust_cmd)
{

	//PX4_INFO("Thrust_CMD: %f", (double)thrust_cmd);

	Vector4f fM;
	fM(0) = thrust_cmd;
	for (size_t i=0; i<3; i++){
		fM(i+1) = torque_cmd(i);
	}

	Vector4f omega;

	if (_jMAVSIM){
		// they assume linear model PWM -> thrust	
	  omega = _invG * fM;
	  for (size_t i=0; i<4; i++){
	  	omega(i) = (omega(i) > 0) ? omega(i) : 0.0;
		}

	} else {
			// gazebo assumes quadratic relationship
		Vector4f omega_sq = _invG * fM;
		for (size_t i=0; i<4; i++){
			omega(i) = (omega_sq(i) > 0) ? sqrt(omega_sq(i)) : 0.0;
		}
	}

	return omega;

}

// INPUT: Vector4f omega, the desired angular speeds of each motor
void MulticopterRateControl::publish_actuator_outputs(Vector4f omega)
{

	float MOTOR_SPEED_MAX = 1.0; // in [0,1]
	if (_jMAVSIM){ 
		MOTOR_SPEED_MAX = 1.0;
	}
	else {
		MOTOR_SPEED_MAX = 2000.0; // TODO: check
	}
	const float PWM_MIN = 1000.0f;
	const float PWM_MAX = 2000.0f;


	// Each acutator output is in range of PWM_MIN:PWM_MAX
	actuator_outputs_s actuator_outputs;

	for (size_t i=0; i<4; i++){
		// assumes linear scaling between PWM and omega output
		float f = omega(i) / MOTOR_SPEED_MAX;
		f = (f > 1) ? 1 : f;
		f = (f < 0) ? 0 : f;
		actuator_outputs.output[i] = PWM_MIN + (PWM_MAX-PWM_MIN) * f;
	}

	//for (size_t i=0; i<4; i++){
	//	actuator_outputs.output[i] = (i==3) ? PWM_MAX : 0.5f*(PWM_MIN + PWM_MAX);
	//}
	
	actuator_outputs.timestamp = hrt_absolute_time();
	actuator_outputs.noutputs = 4;

	_actuator_outputs_pub.publish(actuator_outputs);

}


int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	
	MulticopterRateControl *instance = new MulticopterRateControl();

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

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("indi_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int indi_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
