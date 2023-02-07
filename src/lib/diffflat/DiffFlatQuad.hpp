#pragma once

#include <matrix/math.hpp>

namespace diffflat
{

using namespace matrix;

float sq(float x) {
	return x*x;
}

float cube(float x) {
	return x*x*x;
}

const Matrix3f R_ENU_TO_FRD (Quaternionf(0f, std::sqrt(2.0f)/2.0f, std::sqrt(2.0f)/2.0f, 0.0f) );

void flat_state_ENU_to_quad_state_FRD(
		Vector3f& pos_FRD, 
		Vector3f& vel_FRD, 
		Vector3f& acc_FRD, 
		Vector3f& b1d_FRD, 
		Vector3f& ang_vel_FRD, 
		Vector3f& ang_acc_FRD, 
  const Vector3f pos_ENU, 
  const Vector3f vel_ENU, 
  const Vector3f acc_ENU, 
  const Vector3f jerk_ENU,
  const Vector3f snap_ENU, 
  float yaw, 
  float yaw_rate, 
  float yaw_acc, 
  float g=9.81f
) {

	const Vector3f _pos_FRD = R_ENU_TO_FRD * pos_ENU;
	const Vector3f _vel_FRD = R_ENU_TO_FRD * vel_ENU;
	const Vector3f _acc_FRD = R_ENU_TO_FRD * acc_ENU;

	for (size_t i=0; i < 3; i++){
		pos_FRD(i) = _pos_FRD(i);
		vel_FRD(i) = _vel_FRD(i);
		acc_FRD(i) = _acc_FRD(i);
	}

	Vector3f b1d_ENU;
	Vector3f ang_vel_ENU;
	Vector3f ang_acc_ENU;

	// RUN flat state conversion within ENU
	flat_state_ENU_to_quad_state_ENU(b1d_ENU, ang_vel_ENU, ang_acc_ENU, acc_ENU, jerk_ENU, snap_ENU, yaw, yaw_rate, yaw_acc, g);

	// convert back to FRD
	
	const Vector3f _b1d_FRD = R_ENU_TO_FRD * b1d_ENU;
	const Vector3f _ang_vel_FRD = R_ENU_TO_FRD *  _ang_vel_ENU;
	const Vector3f _ang_acc_FRD = R_ENU_TO_FRD *  _ang_acc_ENU;	
	
	for (size_t i=0; i < 3; i++){
		b1d_FRD(i) = _b1d_FRD(i);
		ang_vel_FRD(i) = _ang_vel_FRD(i);
		ang_acc_FRD(i) = _ang_acc_FRD(i);
	}

}


void flat_state_ENU_to_quad_state_ENU(Vector3f& b1d, Vector3f& ang_vel, Vector3f& ang_acc, 
  const Vector3f acc, 
  const Vector3f jerk,
  const Vector3f snap, 
  float yaw, 
  float yaw_rate, 
  float yaw_acc, 
  float g=9.81f
) {
    
    
    Vector3f force = acc + Vector3f(0, 0, g);

     Vector3f zb = force.unit();
     Vector3f xc(std::cos(yaw), std::sin(yaw), 0.0);
     Vector3f yb = zb.cross(xc).unit();
     Vector3f xb = yb.cross(zb).unit();

    // construct desired rotation matrix
    Dcm<float> R;
    for (size_t i=0; i < 3; i++){
      R(i, 0) = xb(i);
      R(i, 1) = yb(i);
      R(i, 2) = zb(i);
    }
    
    // update b1d
    for (size_t i=0; i < 3; i++){
      // b1d = R * [1,0,0]
      b1d(i) = R(i, 0);
    }

    // construct τ
     float tau = force.norm();

    // construct S matrix
    const float bx1 = R(0, 0);
    const float bx2 = R(1, 0);
    //const float bx3 = R(2, 0);
    const float by1 = R(0, 1);
    const float by2 = R(1, 1);
    //const float by3 = R(2, 1);
    const float bz1 = R(0, 2);
    const float bz2 = R(1, 2);
    //const float bz3 = R(2, 2);
    
    const float bx12 = bx1*bx1;
    const float bx22 = bx2*bx2;

    // S vector
    Vector3f S ( 0, 
        (bx2 * bz1 - bx1 * bz2) / (bx12 + bx22), 
        (-bx2 * by1 + bx1 * by2) / (bx12 + bx22)
    );

    // solve for Ω, τdot

    // bz = R[:, 3] use zb instead

    // construct M matrix
    SqMatrix4 M;

    // M[1:3, 1:3] = tau * R * hatizT;
    Vector3f iz (0, 0, 1);
    Matrix3f hatizT = iz.hat().T();
    Matrix3f M11 = tau * R * hatizT;
    for (size_t i=0; i < 3; i++){
        for (size_t j=0; j < 3; j++) {
            M(i,j) = M11(i,j);
        }
    }
    //M[1:3, 4] = bz
    for (size_t i=0; i < 3; i++){
        M(i, 3) = zb(i);
    }
    // M[4, 1:3] = S
    for (size_t j=0; j < 3; j++ ){
        M(3, j) = S(j);
    }
    // M[4,4] = 0
    M(3,3) = 0.0;

    // get inverse of M
    SqMatrix4 invM = M.I();
    
    //Ωτd = invM * [SVector{3}(j); ψd]
    Vector4f omega_taudot = invM * Vector4f(jerk(0), jerk(1), jerk(2), yaw_rate);

    // update omega
    for (size_t i=0; i<3;i++){
      ang_vel(i) = omega_taudot(i);
    }
   
    const float tau_dot = omega_taudot(3);

    // construct Sdot matrix
    const float w1 = ang_vel(0);
    const float w2 = ang_vel(1);
    const float w3 = ang_vel(2);
    

    Vector3f Sd (
        0,
        (bx1 * w1) / (bx12 + bx22) +
        (bx2 * w2) / (bx12 + bx22) +
        ((bx12 * bz1 - bx22 * bz1 + 2 * bx1 * bx2 * bz2) * w3) / sq(bx12 + bx22),
        ((bx12 * bx2 + cube(bx2) - bx12 * by1 + bx22 * by1 - 2 * bx1 * bx2 * by2) * w3) / sq(bx12 + bx22)
    );

    // solve for α, τdd
    const Vector3f B1 = R * (2 * tau_dot * hatizT + tau * (ang_vel.hat()) * hatizT) * ang_vel;
    const float  B2 = Sd.dot(ang_vel);
    const Vector4f sb (snap(0) - B1(0), snap(1) - B1(1), snap(2) - B1(2), yaw_acc - B2);
    const Vector4f alpha_taudotdot = invM * sb;

    for (size_t i=0; i<3;i++){
      ang_acc(i) = alpha_taudotdot(i);
    }

    return;

}




} // namespace diffflat

