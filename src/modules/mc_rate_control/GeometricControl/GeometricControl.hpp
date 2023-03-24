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
    _J.setIdentity();
	}
	~GeometricControl() = default;

  // functions to set internal parameters
  void set_gains(float kx, float kv, float kR, float komega);
  void set_inertia(float Jxx, float Jyy, float Jzz, float Jxy=0.0f, float Jxz=0.0f, float Jyz=0.0f);

	/**
	 * Run one control loop cycle calculation
   * pass in SI units, returns in SI units
   * Returns [moments, _thrust_sp]
	 */
	matrix::Vector<float, 4> update(
			const matrix::Vector3f &pos, // pass in current state
			const matrix::Vector3f &vel,
			const matrix::Quatf    &ang_att,
			const matrix::Vector3f &ang_rate,
			const vehicle_local_position_setpoint_s &setpoint, // pass in setpoint
                        const vehicle_control_mode_s &control_mode // determines which mode we should control it in
		       	);
	
private:

	const matrix::Vector3f _z = matrix::Vector3f(0,0,1);

	static constexpr float g = 9.81f;
	// float hover_throttle = 0.5f;
	matrix::SquareMatrix<float, 3> _J;

	// gains
	float _kx = 1.0;
	float _kv = 1.0;
	float _kR = 1.0;
	float _kOmega = 1.0;



	matrix::SquareMatrix<float, 3> vector_to_skew(matrix::Vector3f v);
	matrix::Vector3f skew_to_vector(matrix::SquareMatrix<float, 3> M);


};
