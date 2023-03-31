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

#pragma once

#include <RateControl.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/actuator_outputs.h>

// DASC-CUSTOM
// #include <GeometricControl.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/external_controller.h>
#include <uORB/topics/vehicle_attitude.h>


using namespace time_literals;

class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl();
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	void updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt);

	void publishTorqueSetpoint(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};

	vehicle_status_s		_vehicle_status{};

	//matrix::Vector3f moving_error;

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */

	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	hrt_abstime _last_run{0};

  matrix::SquareMatrix<float, 4> _G;
  matrix::SquareMatrix<float, 4> _invG;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INDI_ROLLRATE_P>) _param_indi_rollrate_p,
		(ParamFloat<px4::params::INDI_ROLLRATE_I>) _param_indi_rollrate_i,
		(ParamFloat<px4::params::INDI_RR_INT_LIM>) _param_indi_rr_int_lim,
		(ParamFloat<px4::params::INDI_ROLLRATE_D>) _param_indi_rollrate_d,
		(ParamFloat<px4::params::INDI_ROLLRATE_FF>) _param_indi_rollrate_ff,
		(ParamFloat<px4::params::INDI_ROLLRATE_K>) _param_indi_rollrate_k,

		(ParamFloat<px4::params::INDI_PITCH_P>) _param_indi_pitchrate_p,
		(ParamFloat<px4::params::INDI_PITCH_I>) _param_indi_pitchrate_i,
		(ParamFloat<px4::params::INDI_PR_INT_LIM>) _param_indi_pr_int_lim,
		(ParamFloat<px4::params::INDI_PITCH_D>) _param_indi_pitchrate_d,
		(ParamFloat<px4::params::INDI_PITCH_FF>) _param_indi_pitchrate_ff,
		(ParamFloat<px4::params::INDI_PITCH_K>) _param_indi_pitchrate_k,

		(ParamFloat<px4::params::INDI_YAWRATE_P>) _param_indi_yawrate_p,
		(ParamFloat<px4::params::INDI_YAWRATE_I>) _param_indi_yawrate_i,
		(ParamFloat<px4::params::INDI_YR_INT_LIM>) _param_indi_yr_int_lim,
		(ParamFloat<px4::params::INDI_YAWRATE_D>) _param_indi_yawrate_d,
		(ParamFloat<px4::params::INDI_YAWRATE_FF>) _param_indi_yawrate_ff,
		(ParamFloat<px4::params::INDI_YAWRATE_K>) _param_indi_yawrate_k,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		(ParamFloat<px4::params::INDI_ACRO_R_MAX>) _param_indi_acro_r_max,
		(ParamFloat<px4::params::INDI_ACRO_P_MAX>) _param_indi_acro_p_max,
		(ParamFloat<px4::params::INDI_ACRO_Y_MAX>) _param_indi_acro_y_max,
		(ParamFloat<px4::params::INDI_ACRO_EXPO>) _param_indi_acro_expo,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::INDI_ACRO_EXPO_Y>) _param_indi_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::INDI_SUPEXPO>) _param_indi_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::INDI_SUPEXPOY>) _param_indi_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::INDI_BATSCALEEN>) _param_indi_bat_scale_en,

		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
	
    // // DASC CUSTOM PARAMS
    // (ParamFloat<px4::params::GEO_KX>) _param_geo_kx,
    // (ParamFloat<px4::params::GEO_KV>) _param_geo_kv,
    // (ParamFloat<px4::params::GEO_KR>) _param_geo_kR,
    // (ParamFloat<px4::params::GEO_KOMEGA>) _param_geo_kOmega,
    // (ParamFloat<px4::params::GEO_JXX>) _param_geo_Jxx,
    // (ParamFloat<px4::params::GEO_JYY>) _param_geo_Jyy,
    // (ParamFloat<px4::params::GEO_JZZ>) _param_geo_Jzz,
    // (ParamFloat<px4::params::GEO_JXY>) _param_geo_Jxy,
    // (ParamFloat<px4::params::GEO_JXZ>) _param_geo_Jxz,
    // (ParamFloat<px4::params::GEO_JYZ>) _param_geo_Jyz,
    // (ParamFloat<px4::params::GEO_TORQ_MAX>) _param_geo_torq_max,
    // (ParamFloat<px4::params::GEO_TORQ_CONST>) _param_geo_torq_const,
    // (ParamFloat<px4::params::GEO_HOVER_THR>) _param_geo_hover_thrust
  )
  

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

};
