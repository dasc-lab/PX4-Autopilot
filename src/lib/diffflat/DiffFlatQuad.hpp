#pragma once

#include <matrix/math.hpp>

namespace diffflat
{

using namespace matrix;

template<typename Type>
void flat_state_to_quad_state(Vector<Type, 3>& b1d, Vector<Type, 3>& ang_vel, Vector<Type, 3>& ang_acc, 
  const Vector<Type, 3> acc, 
  const Vector<Type, 3> jerk,
  const Vector<Type, 3> snap, 
  float yaw, 
  float yaw_rate, 
  float yaw_acc, 
  float g
) {
    
    using Vector3 = Vector<Type, 3>;
    using Vector4 = Vector<Type, 4>;
    using SqMatrix3 = SquareMatrix<Type, 3>;
    using SqMatrix4 = SquareMatrix<Type, 4>;

    const Vector3 force(acc(0), acc(1), acc(2) + g);

    const Vector3 zb = force.unit();
    const Vector3 xc(std::cos(yaw), std::sin(yaw), 0);
    const Vector3 yb = zb.cross(xc).unit()
    const Vector3 xb = yb.cross(zb).unit()

    // construct desired rotation matrix
    Dcm<Type> R;
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
    const Type tau = force.norm()

    // construct S matrix
    const Type bx1 = R[0, 0]
    const Type bx2 = R[1, 0]
    const Type bx3 = R[2, 0]
    const Type by1 = R[0, 1]
    const Type by2 = R[1, 1]
    const Type by3 = R[2, 1]
    const Type bz1 = R[0, 2]
    const Type bz2 = R[1, 2]
    const Type bz3 = R[2, 2]

    // S vector
    Vector3 S ( 0, 
        (bx2 * bz1 - bx1 * bz2) / (bx1^2 + bx2^2), 
        (-bx2 * by1 + bx1 * by2) / (bx1^2 + bx2^2)
    )

    // solve for Ω, τdot

    // bz = R[:, 3] use zb instead

    // construct M matrix
    SqMatrix4 M;

    // M[1:3, 1:3] = tau * R * hatizT;
    const Vector3 iz (0, 0, 1);
    const SqMatrix3 hatizT = iz.hat().T();
    const SqMatrix3 M11 = tau * R * hatizT;
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
    Vector4 omega_taudot = invM * Vector4f(jerk(0), jerk(1), jerk(2), yaw_rate);

    // update omega
    for (size_t i=0; i<3;i++){
      ang_vel(i) = omega_taudot(i);
    }
   
    const Type tau_dot = omega_taudot(3);

    // construct Sdot matrix
    const Type w1 = ang_vel(0);
    const Type w2 = ang_vel(1);
    const Type w3 = ang_vel(2);

    Vector3 Sd (
        0,
        (bx1 * w1) / (bx1^2 + bx2^2) +
        (bx2 * w2) / (bx1^2 + bx2^2) +
        ((bx1^2 * bz1 - bx2^2 * bz1 + 2 * bx1 * bx2 * bz2) * w3) / (bx1^2 + bx2^2)^2,
        ((bx1^2 * bx2 + bx2^3 - bx1^2 * by1 + bx2^2 * by1 - 2 * bx1 * bx2 * by2) * w3) / (bx1^2 + bx2^2)^2
    )

    // solve for α, τdd
    const Vector3 B1 = R * (2 * tau_dot * hatizT + tau * (ang_vel.hat()) * hatizT) * ang_vel
    const Type  B2 = Sd.dot(ang_vel)
    const Vector4 sb (s(0) - B1(0), s(1) - B1(1), s(2) - B1(2), yaw_acc - B2);
    const Vector4 alpha_taudotdot = invM * sb

    for (size_t i=0; i<3;i++){
      ang_acc(i) = alpha_taudotdot(i);
    }

    return;

}



} // namespace diffflat

