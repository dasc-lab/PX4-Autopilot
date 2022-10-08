/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file GeometricControl.hpp
 *
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>


//#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
//#include <uORB/topics/rate_ctrl_status.h>

class GeometricControl
{
public:
	GeometricControl(){
		J.setIdentity();
	}
	~GeometricControl() = default;

	/**
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	//void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	//void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	//void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	//void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
	//			 const matrix::Vector<bool, 3> &saturation_negative);


	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Matrix<float, 4, 1> update(
			const matrix::Vector3f &pos, // pass in current state
			const matrix::Vector3f &vel,
			const matrix::Quatf    &ang_att,
			const matrix::Vector3f &ang_rate,
			const vehicle_local_position_setpoint_s &setpoint, // pass in setpoint
                        const vehicle_control_mode_s &control_mode // determines which mode we should control it in
		       	);
	
	
	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
 */	//void resetIntegral() { _rate_int.zero(); }


private:

	matrix::Vector3f _z = matrix::Vector3f(0,0,1);

	float m = 1.0;
	float g = 9.81;
	float hover_throttle = 0.5;
	matrix::SquareMatrix<float, 3> J;
	float torque_constant = 1.0;
	float torque_max = 1.0;

	// gains
	float kx = 1.0;
	float kv = 1.0;
	float kR = 1.0;
	float kOmega = 1.0;



	matrix::SquareMatrix<float, 3> vector_to_skew(matrix::Vector3f v);
	matrix::Vector3f skew_to_vector(matrix::SquareMatrix<float, 3> M);

	//void updateIntegral(matrix::Vector3f &rate_error, const float dt);
	// Gains
	//matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	//matrix::Vector3f _gain_i; ///< rate control integral gain
	//matrix::Vector3f _gain_d; ///< rate control derivative gain
	//matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	//matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	//matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	//matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	//matrix::Vector<bool, 3> _control_allocator_saturation_positive;
};
