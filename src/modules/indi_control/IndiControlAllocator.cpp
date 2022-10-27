#include "IndiControl.hpp"

using namespace matrix;

void IndiControl::update_allocator_rpm_bounds(const float min_rpm,
                                              const float max_rpm) {

  c_float l_new[4] = {min_rpm, min_rpm, min_rpm, min_rpm};
  c_float u_new[4] = {max_rpm, max_rpm, max_rpm, max_rpm};

  // update bounds
  c_int res = osqp_update_bounds(&workspace, l_new, u_new);
  if (res != 0) {
    PX4_WARN("unable to update bounds");
  }
}

void IndiControl::update_allocator_matrices(const Matrix4f H,
                                            const Matrix4f G) {

  _H = H;
  _G = G;

  Matrix4f GHG = _G.transpose() * _H * _G;

  // update the CSC variables
  c_float P_x_new[10];
  int ind = 0;
  for (size_t j = 0; j < 4; j++) {
    for (size_t i = 0; i <= j; i++) {
      P_x_new[ind] = GHG(i, j);
      ind++;
    }
  }

  c_int res = osqp_update_P(&workspace, P_x_new, OSQP_NULL, 10);
  if (res != 0) {
    PX4_WARN("unable to update quadratic cost");
  }
}

Vector4f IndiControl::solve_allocator_qp(const Vector4f mu_ref) {

  // construct the linear cost
  Vector4f q = -_G.transpose() * _H * mu_ref;

  c_float q_new[4];
  for (size_t i = 0; i < 4; i++) {
    q_new[i] = q(i);
  }

  // update linear cost
  c_int res = osqp_update_lin_cost(&workspace, q_new);
  if (res != 0) {
    PX4_WARN("unable to update linear cost");
  }

  // solve
  res = osqp_solve(&workspace);
  if (res != 0) {
    PX4_WARN("SOLVE EXIT FLAG != 0");
  }

  // check status
  if (workspace.info->status_val != OSQP_SOLVED) {
    PX4_WARN("OSQP DIDNT SOLVE SUCCESSFULLY! RETURNED: %d",
             workspace.info->status_val);
  }

  // construct solution
  Vector4f omegas;
  for (size_t i = 0; i < 4; i++) {
    omegas(i) = workspace.solution->x[i];
  }

  // Vector4f mu_out = _G * omegas;

  // PX4_INFO("IN: %f, %f, %f, %f, OUT: %f, %f, %f, %f",
  //     (double)mu_ref(0),
  //     (double)mu_ref(1),
  //     (double)mu_ref(2),
  //     (double)mu_ref(3),
  //     (double)mu_out(0),
  //     (double)mu_out(1),
  //     (double)mu_out(2),
  //     (double)mu_out(3)
  //     );

  // PX4_INFO("MU_DIFF: %f, %f, %f, %f",
  //    (double)(mu_out(0) - mu_ref(0)),
  //    (double)(mu_out(1) - mu_ref(1)),
  //    (double)(mu_out(2) - mu_ref(2)),
  //    (double)(mu_out(3) - mu_ref(3))
  //    );
  return omegas;
}
