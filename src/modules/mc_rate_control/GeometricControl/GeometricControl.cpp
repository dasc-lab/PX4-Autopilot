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
 * @file GeometricControl.cpp
 */

#include <GeometricControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

// void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
// {
// 	_gain_p = P;
// 	_gain_i = I;
// 	_gain_d = D;
// }
// 
// void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
// 				      const Vector<bool, 3> &saturation_negative)
// {
// 	_control_allocator_saturation_positive = saturation_positive;
// 	_control_allocator_saturation_negative = saturation_negative;
// }

matrix::Matrix<float, 4, 1> GeometricControl::update(
			const matrix::Vector3f &pos, // pass in current state
			const matrix::Vector3f &vel,
			const matrix::Quatf    &ang_att,
			const matrix::Vector3f &ang_rate,
			const vehicle_local_position_setpoint_s &setpoint, // pass in setpoint
                        const vehicle_control_mode_s &control_mode // determines which mode we should control it in
		       	)
{

	// actual implementation of the geometric controller
	

	// error in position
	const matrix::Vector3f pos_sp(setpoint.x, setpoint.y, setpoint.z);
	const matrix::Vector3f pos_err = pos - pos_sp;

	// error in velocity
	const matrix::Vector3f vel_sp(setpoint.vx, setpoint.vy, setpoint.vz);
	const matrix::Vector3f vel_err = vel - vel_sp;
	
	const matrix::Vector3f acc_sp (setpoint.acceleration[0], setpoint.acceleration[1], setpoint.acceleration[2]);
	
	// create the required acceleration vector
	const matrix::Vector3f acc = -(-kx* pos_err -kv*vel_err - g*_z + acc_sp);

	// get the unit vector in the desired thrust direction
	const matrix::Vector3f b3d = (acc.norm() < 0.01f) ? Vector3f(0,0,1) : acc.unit();

	// get the unit vector pointing in the heading direction
	
	float b1dx = cosf(setpoint.yaw);
	float b1dy = sinf(setpoint.yaw);
	float b1dz = 0.0f;

	const matrix::Vector3f b1d(b1dx, b1dy, b1dz);

	// get the unit vector b2d
	const matrix::Vector3f b2d = b3d.cross(b1d).unit();

	// recompute the b1d
	const matrix::Vector3f b1d_new = b2d.cross(b3d).unit();


	// construct the desired rotation matrix
	matrix::Dcm<float> rotDes;
	for (uint i=0; i< 3; i++){
		rotDes(i,0) = b1d_new(i);	
		rotDes(i,1) = b2d(i);	
		rotDes(i,2) = b3d(i);
	}	


	// construct the current rotation matrix
	const matrix::Dcm<float> rotMat(ang_att);
	
	// error in rotation
	matrix::Vector3f rotMat_err = 0.5f * skew_to_vector( rotDes.T() * rotMat - rotMat.T() * rotDes);
	
	// grab angular velocity and acceleration setpoints
	const Vector3f ang_rate_sp(0,0,0);
	const Vector3f ang_acc_sp(0,0,0);
	
	// error in angular rates
	const Vector3f ang_rate_err = ang_rate - rotMat.T() * rotDes * ang_rate_sp;

	// compute the desired total thrust
	const float coll_acc = acc.dot( rotMat * _z);

	// compute the desired moments
	const matrix::Vector3f omega_cross_Jomega = ang_rate.cross ( J * ang_rate);

	const matrix::Vector3f moments = -kR * rotMat_err - kOmega * ang_rate_err
		+ omega_cross_Jomega
		- J * (vector_to_skew(ang_rate) * rotMat.T() * rotDes * ang_rate_sp); // technically should have additional angular acceleration terms - ignored in this implementation. 

	// get the angular acceleration
	
	const matrix::Vector3f ang_acc_des = J.I() * ( moments - omega_cross_Jomega );

	
	// normalize terms - convert from SI into arbitrary ranges for the drones
	matrix::Vector3f ang_acc_des_normalized = ang_acc_des / torque_constant;
	float coll_thrust_normalized = math::min(1.0f, math::max(0.0f, coll_acc / g * hover_throttle)); // clamp the thrust to a reasonable rangle


	// bound the max angular acceleration
	if ( ang_acc_des_normalized.norm() > torque_max) {
          ang_acc_des_normalized = ang_acc_des_normalized.unit() * torque_max;
	}	



	// return things
	matrix::Matrix<float, 4,1> res;
	res(0,0) = coll_thrust_normalized;
	res(1,0) = ang_acc_des_normalized(0);
	res(2,0) = ang_acc_des_normalized(1);
	res(3,0) = ang_acc_des_normalized(2);
	
	return res;

}


matrix::SquareMatrix<float, 3> GeometricControl::vector_to_skew(matrix::Vector3f v){

	matrix::SquareMatrix<float, 3> M;
	M(0,0) = 0;
	M(0,1) = -v(2);
	M(0,2) = v(1);
	
	M(1,0) = v(2);
	M(1,1) = 0;
	M(1,2) = -v(0);

	M(2,0) = -v(1);
	M(2,1) = v(0);
	M(2,2) = 0;

	return M;
}

matrix::Vector3f GeometricControl::skew_to_vector(matrix::SquareMatrix<float, 3> M){

	matrix::Vector3f v;
	v(0) = 0.5f * (M(2,1) - M(1,2));
	v(1) = 0.5f * (M(0,2) - M(2,0));
	v(2) = 0.5f * (M(1,0) - M(0,1));

	return v;
}
