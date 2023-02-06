// Indi Controller
// Devansh Agrawal Oct 2022

#include "IndiControl.hpp"

using namespace matrix;

IndiControl::IndiControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {

  // set default setpoint
  _setpoint.pos[0] = 0.0;
  _setpoint.pos[1] = 0.0;
  _setpoint.pos[2] = -1.25;
  _setpoint.yaw = M_PI * 0.5;

  // set up filters
  _tau_bz_filter.set_cutoff_frequency(RATE_TAU_BZ, RATE_LPF);
  _ang_vel_filter.set_cutoff_frequency(RATE_ANG_VEL, RATE_LPF);
  _ang_accel_filter.set_cutoff_frequency(RATE_ANG_ACC, RATE_LPF);
  _a_filter.set_cutoff_frequency(RATE_ACCEL, RATE_LPF);
  _torque_filter.set_cutoff_frequency(RATE_TORQUE, RATE_LPF);
  _manual_control_z_filter.set_cutoff_frequency(RATE_TORQUE, RATE_MANUAL_Z);
}

IndiControl::~IndiControl() {
  perf_free(_loop_perf);
  perf_free(_loop_interval_perf);
}

bool IndiControl::init() {

  update_parameters();

  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    return false;
  }

  // Run on fixed interval
  _interval = (uint32_t)(1.0e6 / ((double)RATE_TORQUE));
  // ScheduleOnInterval(_interval); // 500 us interval, 2000 Hz rate // TODO:
  // Make this a parameter

  return true;
}

void IndiControl::update_parameters() {

  // update all the constants
  _mass = _param_mass.get();
  _thrust_constant = _param_thrust_constant.get();
  _torque_constant = _param_torque_constant.get();
  _J(0, 0) = _param_Jxx.get();
  _J(0, 1) = _param_Jxy.get();
  _J(0, 2) = _param_Jxz.get();
  _J(1, 0) = _param_Jxy.get();
  _J(1, 1) = _param_Jyy.get();
  _J(1, 2) = _param_Jyz.get();
  _J(2, 0) = _param_Jxz.get();
  _J(2, 1) = _param_Jyz.get();
  _J(2, 2) = _param_Jzz.get();

  const float lx = 0.13;
  const float ly = 0.22;
  // assuming the motors are in the pattern (body FRD frame)
  //   +bx
  //    ^
  //    |
  //    |
  //
  //  2   0
  //    x    --> +by
  //  1   3
  //  and motor 0 spins ccw (so neg bz direction)

  Vector4f motor_pos_x(lx, -lx, lx, -lx);
  Vector4f motor_pos_y(ly, -ly, -ly, ly);
  Vector4f motor_dir(
      1, 1, -1, -1); // positive for torque in +bz direction (body FRD frame)

  // construct the effectiveness matrix
  Matrix4f G;
  G.zero();
  for (size_t i = 0; i < 4; i++) {
    // torque about body x:
    G(0, i) = -motor_pos_y(i) * _thrust_constant;
    // torque about body y:
    G(1, i) = motor_pos_x(i) * _thrust_constant;
    // torque about body z:
    G(2, i) = motor_dir(i) * _torque_constant * _thrust_constant;
    // thrust in body z:
    G(3, i) = _thrust_constant;
  }

  // construct the cost matrix for the allocator QP
  Matrix4f H;
  H.zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 0.1;
  H(3, 3) = 0.5;

  // update the matrices in the allocator
  update_allocator_matrices(H, G);

  // set rpm bounds
  update_allocator_rpm_bounds(0.0, _max_motor_rpm);
}

void IndiControl::service_subscriptions() {

  cb_params();
  cb_vehicle_status();
  cb_vehicle_control_mode();
  cb_manual_control_setpoint();
  cb_vehicle_local_position();
  cb_vehicle_attitude();
  cb_vehicle_angular_velocity();
  cb_vehicle_angular_acceleration();
  cb_sensor_accel();
  cb_diffflat_setpoint();
}

void IndiControl::compute_cmd_accel() {

  Vector3f k_pos(3, 3, 2);
  Vector3f k_vel(1, 1, 0.75);
  Vector3f k_acc(0.0, 0.0, 0.0);

  Vector3f x(_local_position.x, _local_position.y, _local_position.z);
  Vector3f v(_local_position.vx, _local_position.vy, _local_position.vz);

  Vector3f x_ref(_setpoint.pos);
  Vector3f v_ref(_setpoint.vel);
  Vector3f a_ref(_setpoint.acc);

  // acceleration command (EZRA, EQ:17)
  _a_cmd = a_ref.zero_if_nan() + k_pos.emult((x_ref - x).zero_if_nan()) +
           k_vel.emult((v_ref - v).zero_if_nan()) +
           k_acc.emult((a_ref - _a_filt).zero_if_nan());
}

void IndiControl::compute_cmd_thrust() {
  bool use_geometric = false;

  if (use_geometric) {

    float g = 9.81;
    Vector3f _z(0, 0, 1);

    Vector3f acc = _a_cmd - g * _z;

    float coll_acc = acc.dot(Quatf(_attitude.q).rotateVector(_z));
    _thrust_cmd = -coll_acc * _mass;
  } else {
    // EZRA, EQ:20
    _tau_bz_cmd = _tau_bz_filt + _a_cmd - _a_filt;

    // EZRA, EQ:21
    _thrust_cmd = _mass * _tau_bz_cmd.norm();
  }
}

void IndiControl::compute_cmd_thrust_manual() {

  const float max_thrust =
      0.75f * 4.0f * _thrust_constant * _max_motor_rpm * _max_motor_rpm;

  _thrust_cmd =
      _manual_control_z_filter.apply(_manual_control_setpoint.z) * max_thrust;
}

void IndiControl::compute_cmd_quaternion() {
  bool use_geometric = true;
  if (use_geometric) {
    const float g = 9.81;
    Vector3f _z(0, 0, 1);

    Vector3f acc = _a_cmd - g * _z;

    Vector3f b3d = (acc.norm() < 0.01f) ? _z : -acc.unit();

    float yaw_ref = _setpoint.yaw;

    float b1dx = cosf(yaw_ref);
    float b1dy = sinf(yaw_ref);
    float b1dz = 0.0f;

    const Vector3f b1d(b1dx, b1dy, b1dz);
    const Vector3f b2d = b3d.cross(b1d).unit();
    const Vector3f b1d_new = b2d.cross(b3d).unit();

    // construct desired rot matrix
    Dcm<float> rotDes;
    for (size_t i = 0; i < 3; i++) {
      rotDes(i, 0) = b1d_new(i);
      rotDes(i, 1) = b2d(i);
      rotDes(i, 2) = b3d(i);
    }

    _xi_cmd = Quatf(rotDes);
  } else {

    // get body to inertial quaterion
    Quatf xi(_attitude.q);

    // EZRA, EQ:22
    Vector3f neg_bz_cmd_body = xi.rotateVectorInverse(-_tau_bz_cmd);

    // EZRA, EQ:23
    Vector3f iz(0, 0, 1);
    Quatf xi_bar(iz, neg_bz_cmd_body); // min rotation constructor (defined in
                                       // matrix/Quaternion.hpp)

    // EZRA, EQ:24
    const float yaw_ref = _setpoint.yaw;
    Vector3f heading_vec_inertial(std::sin(yaw_ref), -std::cos(yaw_ref), 0);
    Quatf xi_xi_bar = xi * xi_bar;
    Vector3f heading_vec_bar =
        xi_xi_bar.rotateVectorInverse(heading_vec_inertial);

    // EZRA, EQ:25
    Quatf xi_yaw(1, 0, 0, -heading_vec_bar(0) / heading_vec_bar(1));
    xi_yaw.normalize();

    // EZRA, EQ:26
    _xi_cmd = xi_bar * xi_yaw;
  }
  // publish_xi_cmd();
}

void IndiControl::compute_cmd_quaternion_manual() {

  const float g = 9.81;
  const Vector3f _z(0, 0, 1);

  const float max_angle = 30.0f;

  const float max_acc = g * std::tan(max_angle * ((float)M_PI) / 180.f);

  const float max_yaw_rate = 2.0f * (float)M_PI / 8.0f;

  // update yaw setpoint
  _manual_yaw_setpoint +=
      _manual_control_setpoint.r * max_yaw_rate / RATE_TORQUE;

  // determine desired roll and pitch
  float p_acc = _manual_control_setpoint.x * max_acc;
  float r_acc = _manual_control_setpoint.y * max_acc;

  float c = cosf(_manual_yaw_setpoint);
  float s = sinf(_manual_yaw_setpoint);

  const Vector3f acc =
      Vector3f(c * p_acc - s * r_acc, s * p_acc + c * r_acc, -g);

  // get desired body z axis
  Vector3f b3d = (acc.norm() < 0.01f) ? _z : -acc.unit();

  float b1dx = cosf(_manual_yaw_setpoint);
  float b1dy = sinf(_manual_yaw_setpoint);
  float b1dz = 0.0f;

  const Vector3f b1d(b1dx, b1dy, b1dz);
  const Vector3f b2d = b3d.cross(b1d).unit();
  const Vector3f b1d_new = b2d.cross(b3d).unit();

  // construct desired rot matrix
  Dcm<float> rotDes;
  for (size_t i = 0; i < 3; i++) {
    rotDes(i, 0) = b1d_new(i);
    rotDes(i, 1) = b2d(i);
    rotDes(i, 2) = b3d(i);
  }

  // convert to quaternion
  _xi_cmd = Quatf(rotDes);
}

void IndiControl::compute_cmd_ang_accel() {

  bool use_geometric = true;

  if (use_geometric) {

    const Vector3f kR(1.0, 1.0, 0.5);
    const Vector3f kOmega(0.125, 0.125, 0.125 / 2.0);

    // get the desired rotation matrix
    Dcm<float> rotDes(_xi_cmd);

    // construct current rot matrix
    Dcm<float> rotMat(Quatf(_attitude.q));

    // get rotation error
    Vector3f rotMat_err =
        0.5f *
        ((Dcm<float>)((rotDes.T() * rotMat - rotMat.T() * rotDes))).vee();

    // error in angular rate
    Vector3f ang_rate(_ang_vel);
    Vector3f ang_rate_sp(0, 0, 0);
    Vector3f ang_rate_err = ang_rate - rotMat.T() * rotDes * ang_rate_sp;

    if (false) {
      // desired moments
      Vector3f omega_cross_Jomega = ang_rate.cross(_J * ang_rate);

      Vector3f moments =
          -kR.emult(rotMat_err) - kOmega.emult(ang_rate_err) +
          omega_cross_Jomega -
          _J * (ang_rate.hat() * rotMat.T() * rotDes * ang_rate_sp);

      _ang_accel_cmd = _J.I() * (moments - omega_cross_Jomega);

    } else {

      _ang_accel_cmd =
          _J.I() * (-kR.emult(rotMat_err) - kOmega.emult(ang_rate_err));
    }

  } else {
    // // EZRA, EQ:27
    // float xi_w = _xi_cmd(0);
    // Vector3f xi_e = (2.0f * std::acos(xi_w) / std::sqrt(1.0f - xi_w * xi_w))
    // * _xi_cmd.imag();

    // Vector3f k_xi(175, 175, 82);
    // Vector3f k_omega(19.5, 19.5, 19.2);

    // k_xi *= 0.01;
    // k_omega *= 0.01;

    // Vector3f ang_vel_ref(0, 0, 0); //TODO: FIX
    // Vector3f ang_accel_ref(0, 0, 0); // TODO: FIX

    // // EZRA, EQ:28
    // _ang_accel_cmd = ang_accel_ref.zero_if_nan()
    // 		 + k_xi.emult(xi_e.zero_if_nan())
    // 		 + k_omega.emult((ang_vel_ref - _ang_vel_filt).zero_if_nan());
    // // why do i need the filtered version?
  }
  // publish_ang_accel_cmd();
}

void IndiControl::compute_cmd_torque() {

  // EZRA, EQ:31
  _torque_cmd = _torque_filt.zero_if_nan() +
                _J * ((_ang_accel_cmd - _ang_accel).zero_if_nan());
}

void IndiControl::compute_cmd_pwm() {

  // note RPM is actually in units of (1000 radians) per second

  // translate from thrust and torque setpoints to motor RPM
  Vector4f mu_T(_torque_cmd(0), _torque_cmd(1), _torque_cmd(2), _thrust_cmd);

  Vector4f rpm_sq = solve_allocator_qp(mu_T);

  // catch negatives
  for (size_t i = 0; i < 4; i++) {
    if (rpm_sq(i) < 0.0f) {
      rpm_sq(i) = 0.0f;
    }
  }

  // translate from motor RPM to motor PWM
  Vector4f rpm = rpm_sq.sqrt(); // in units of kilo-rad/s

  // map to RPM
  for (size_t i = 0; i < 4; i++) {
    _pwm_cmd(i) = rpm(i) / _max_motor_rpm;
    _pwm_cmd(i) = math::constrain(_pwm_cmd(i), -1.0f, 1.0f);
  }

  // update filters
  Vector4f mu_T_applied = _G * rpm_sq;

  // // update tau_bz filter
  float thrust = mu_T_applied(3);
  Vector3f tau_bz = -(thrust / _mass) * _bz;
  if (!tau_bz.has_nan()) {
    _tau_bz_filt = _tau_bz_filter.apply(tau_bz);
  }
  // update torque filt
  Vector3f torque_applied;
  for (size_t i = 0; i < 3; i++) {
    torque_applied(i) = mu_T_applied(i);
  }
  if (!torque_applied.has_nan()) {
    _torque_filt = _torque_filter.apply(torque_applied);
  }

  // update accel filter
  if (!_a.has_nan()) {
    _a_filt = _a_filter.apply(_a);
  }

  publish_pwm_cmd();
}

bool IndiControl::check_flight_mode() {

  if (_vehicle_control_mode.flag_control_manual_enabled &&
      !_vehicle_control_mode.flag_control_altitude_enabled &&
      !_vehicle_control_mode.flag_control_velocity_enabled &&
      !_vehicle_control_mode.flag_control_position_enabled) {
    // is in stabilized mode
    return true;
  } else {
    return false;
  }
}

void IndiControl::compute_cmd() {

  bool mode_manual = check_flight_mode();

  if (!mode_manual) {
    // run the main controller
    compute_cmd_accel();
    compute_cmd_thrust();
    compute_cmd_quaternion();
  } else {
    compute_cmd_thrust_manual();
    compute_cmd_quaternion_manual();
  }
  compute_cmd_ang_accel();
  compute_cmd_torque();
  compute_cmd_pwm();

  // // print pos error
  // Vector3f x(_local_position.x, _local_position.y, _local_position.z);
  // Vector3f x_ref(_setpoint.position);
  // if (!(x - x_ref).has_nan() && _armed) {
  //   running_pos_err.update((x - x_ref));
  //   // Vector3f mu = running_pos_err.mean();
  //   // Vector3f sigma = running_pos_err.variance().sqrt();

  //   if (running_pos_err.count() > 10000) {
  //     running_pos_err.reset();
  //   }

  //   // PX4_INFO("POS ERROR: %f, %f, %f",
  //   //     (double)(x-x_ref)(0),
  //   //     (double)(x-x_ref)(1),
  //   //     (double)(x-x_ref)(2));

  //   // PX4_INFO("POS ERROR: (%f +- %f)   (%f +- %f)   (%f +- %f)",
  //   (double)mu(0),
  //   //         (double)sigma(0), (double)mu(1), (double)sigma(1),
  //   (double)mu(2),
  //   //         (double)sigma(2));
  // }
}

void IndiControl::construct_setpoint() {
  float time_since_start_s = (float)(_now - _start) * 1.0e-6f;
  float seconds_per_rev = 5.0f;

  float omega = 2.0f * (float)M_PI / seconds_per_rev;
  float phase = omega * time_since_start_s;

  float amplitude = 0.5f;

  _setpoint.pos[0] = amplitude * cos(phase);
  _setpoint.pos[1] = amplitude * sin(phase);
  _setpoint.pos[2] = -1.25f;

  _setpoint.vel[0] = -amplitude * omega * sin(phase);
  _setpoint.vel[1] = amplitude * omega * cos(phase);
  _setpoint.vel[2] = 0.0f;

  _setpoint.acc[0] = -amplitude * omega * omega * cos(phase);
  _setpoint.acc[1] = -amplitude * omega * omega * sin(phase);
  _setpoint.acc[2] = 0.0f;

  _setpoint.yaw = -0.0f * 0.5f * phase;
}

void IndiControl::Run() {
  if (should_exit()) {
    ScheduleClear();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);
  perf_count(_loop_interval_perf);

  _now = hrt_absolute_time();
  service_subscriptions();
  construct_setpoint();
  compute_cmd();

  // PX4_INFO("PERIOD: %lu INTERVAL: %u", (_now - _last), _interval);
  _last = _now;

  perf_end(_loop_perf);
}

int IndiControl::task_spawn(int argc, char *argv[]) {
  IndiControl *instance = new IndiControl();

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

int IndiControl::print_status() {
  perf_print_counter(_loop_perf);
  perf_print_counter(_loop_interval_perf);
  return 0;
}

int IndiControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int IndiControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Indi Controller
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("indi_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int indi_control_main(int argc, char *argv[]) {
  return IndiControl::main(argc, argv);
}
