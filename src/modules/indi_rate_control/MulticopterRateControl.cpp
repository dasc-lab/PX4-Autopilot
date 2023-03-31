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

MulticopterRateControl::MulticopterRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	//_controller_status_pub.advertise();

	//moving_error(0) = 0.0;
	//moving_error(1) = 0.0;
	//moving_error(2) = 0.0;
	

	construct_G_matrix();

}

void MulticopterRateControl::construct_G_matrix()
{
  
	// construct the G, Ginv matrix,
	// assumes the quadrotor is arranged as follows:
	//
	//   3     1
	//      x
	//   2     4
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

	float LX = 0.3;
	float LY = 0.3;
	float k_thrust = 1.0;
	float k_torque = 0.6;

	for (size_t motor_ind = 0; motor_ind < 4; motor_ind ++){

		_G(0, motor_ind) = -k_thrust;
	// TODO: CONTINUE HERE	


	}

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
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_indi_rollrate_k.get(), _param_indi_pitchrate_k.get(), _param_indi_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_indi_rollrate_p.get(), _param_indi_pitchrate_p.get(), _param_indi_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_indi_rollrate_i.get(), _param_indi_pitchrate_i.get(), _param_indi_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_indi_rollrate_d.get(), _param_indi_pitchrate_d.get(), _param_indi_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_indi_rr_int_lim.get(), _param_indi_pr_int_lim.get(), _param_indi_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_indi_rollrate_ff.get(), _param_indi_pitchrate_ff.get(), _param_indi_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_indi_acro_r_max.get()), radians(_param_indi_acro_p_max.get()),
				  radians(_param_indi_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);


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

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// GRAB VEHICLE_ANGULAR_ACCEL (immediately)
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);
		const hrt_abstime now = angular_velocity.timestamp_sample;


		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		_vehicle_status_sub.update(&_vehicle_status);

		// GRAB rates setpoint
		vehicle_rates_setpoint_s v_rates_sp;
		if (_v_rates_sp_sub.update(&v_rates_sp)) {
			_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
			_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
			_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
			_thrust_sp = -v_rates_sp.thrust_body[2];
		}
		
		// Run the rate controller
		
		float _kp = 1.0;
		float _kd = 1.7;
		Vector3f torque_cmd = _rates_sp + _kp * (_rates_sp - rates) - _kd * (angular_accel);

		//Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, false);
		//moving_error = 0.99f * moving_error + 0.01f * (rates - _rates_sp);

		// Do the mixing
		Vector4f omega = mix(torque_cmd, _thrust_sp);

		publish_actuator_outputs(omega);
	
	}

	perf_end(_loop_perf);
}


Vector4f MulticopterRateControl::mix(Vector3f torque_cmd, float thrust_cmd)
{

	Vector4f fM (thrust_cmd, torque_cmd(0), torque_cmd(1), torque_cmd(2));

	Vector4f omega = _Ginv * fM;

	return omega

}

// INPUT: Vector4f omega, the desired angular speeds of each motor
void MulticopterRateControl::publish_actuator_outputs(Vector4f omega)
{

	const float MOTOR_SPEED_MAX = 2000.0;
	const float PWM_MIN = 1000.0f;
	const float PWM_MAX = 2000.0f;


	// Each acutator output is in range of PWM_MIN:PWM_MAX
	actuator_outputs_s actuator_outputs;

	for (size_t i=0; i<4; i++){
		// assumes linear scaling between PWM and omega output
		actuator_outputs.output[i] = PWM_MIN + (PWM_MAX-PWM_MIN) * (omega(i) / MOTOR_SPEED_MAX);
	}
	
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
