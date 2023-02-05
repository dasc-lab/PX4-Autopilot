#include "GeometricControl.hpp"

#include <circuit_breaker/circuit_breaker.h>
#include <drivers/drv_hrt.h>
//#include <mathlib/math/Limits.hpp>
//#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

GeometricControl::GeometricControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {
  
  _J.setIdentity();
  _R.setIdentity();
  parameters_updated();
  
  R_ENU_TO_FRD(0,0) = 0.0f;
  R_ENU_TO_FRD(1,0) = 1.0f;
  R_ENU_TO_FRD(2,0) = 0.0f;
  R_ENU_TO_FRD(0,1) = 1.0f;
  R_ENU_TO_FRD(1,1) = 0.0f;
  R_ENU_TO_FRD(2,1) = 0.0f;
  R_ENU_TO_FRD(0,2) = 0.0f;
  R_ENU_TO_FRD(1,2) = 0.0f;
  R_ENU_TO_FRD(2,2) =-1.0f;

}

GeometricControl::~GeometricControl() { perf_free(_loop_perf); }

bool GeometricControl::init() {
  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }

  return true;
}


void GeometricControl::parameters_updated() {

  // update controller gains
  set_gains(_param_geo_kx.get(), _param_geo_kv.get(), _param_geo_kR.get(), _param_geo_kOmega.get());

  // update motor params
  _k_f = _param_geo_k_f.get(); // N / kRPM^2
  _k_mu = _param_geo_k_mu.get(); // Nm / kRPM^2
  _omega_max = _param_geo_omega_max.get(); // kilo-RPM (this unit is used to reduce the condition number of G)
  
  // update quad inertia
  _mass = _param_geo_mass.get();
  set_inertia(_param_geo_Jxx.get(), _param_geo_Jyy.get(), _param_geo_Jzz.get(), 0, 0, 0); // TODO: update to include cross terms
  
  // update motor pos
  Vector4f motor_pos_x ( _param_geo_M1x.get(), _param_geo_M2x.get(),_param_geo_M3x.get(),_param_geo_M4x.get());
  Vector4f motor_pos_y ( _param_geo_M1y.get(), _param_geo_M2y.get(),_param_geo_M3y.get(),_param_geo_M4y.get());
  Vector4f motor_dir ( 1, 1, -1, -1);

  // update G matrix
  Matrix4f G;
  for (size_t i=0; i<4; i++){
	  G(0, i) = _k_f;
	  Vector3f p (motor_pos_x(i), motor_pos_y(i), 0.0); // TODO: maybe make this more generic?
	  Vector3f mu = p.cross(Vector3f(0,0,_k_f));
	  G(1, i) = mu(0);
	  G(2, i) = mu(1);
	  G(3, i) = -motor_dir(i) * _k_mu;
  }

  // update inv G matrix
  _invG = G.I();

}

void GeometricControl::update_state() { 

  // check EKF
  if (_vehicle_local_position_sub.updated()) {
	  vehicle_local_position_s local_position {};
    _vehicle_local_position_sub.copy(&local_position);

    _x(0) =local_position.xy_valid ?  local_position.y : NAN;
    _x(1) =local_position.xy_valid ?  local_position.x : NAN;
    _x(2) =local_position.z_valid ?  -local_position.z : NAN;
    
    _v(0) =local_position.v_xy_valid ?  local_position.vy : NAN;
    _v(1) =local_position.v_xy_valid ?  local_position.vx : NAN;
    _v(2) =local_position.v_z_valid ?  -local_position.vz : NAN;


    _a(0) = local_position.ay;
    _a(1) = local_position.ax;
    _a(2) = -local_position.az;

  }

  // check attitude
  if (_vehicle_attitude_sub.updated()) {
	  vehicle_attitude_s vehicle_attitude{};
	  _vehicle_attitude_sub.copy(&vehicle_attitude);
	  Dcmf R_FRD (Quaternionf(vehicle_attitude.q));
	  // now convert to ENU
	  _R = R_ENU_TO_FRD.T() * R_FRD * R_ENU_TO_FRD;

	  Quaternion<float> q_enu(_R);
	  //PX4_WARN("NEW ROTATION: %.3f, %.3f, %.3f, %.3f", (double)vehicle_attitude.q[0], (double)vehicle_attitude.q[1],(double)vehicle_attitude.q[2],(double)vehicle_attitude.q[3]); 
	  PX4_WARN("NEW ROTATION: %.3f, %.3f, %.3f, %.3f", (double)q_enu(0), (double)q_enu(1),(double)q_enu(2),(double)q_enu(3)); 
  }

}

void GeometricControl::update_setpoint() {

		diffflat_setpoint_s sp;
	if (_diffflat_setpoint_sub.updated()){

		_diffflat_setpoint_sub.update(&sp);

		_xd.copy_in(sp.pos);
		_vd.copy_in(sp.vel);
		_ad.copy_in(sp.acc);

	}
		// HACK
		_xd(0) = 0.0;
		_xd(1) = 0.0;
		_xd(2) = 1.0;
		for (size_t i=0; i<3; i++){
			_vd(i) = 0;
			_ad(i) = 0;
			sp.acc[i] = 0;
			sp.jerk[i] = 0;
			sp.snap[i] = 0.0;
			sp.yaw = 0.0;
			sp.yaw_rate = 0;
			sp.yaw_acc = 0;
		}

		diffflat::flat_state_to_quad_state(_b1d, _Omegad, _alphad, 
				Vector3f(sp.acc),
				Vector3f(sp.jerk),
			       	Vector3f(sp.snap),
			       	sp.yaw, sp.yaw_rate, sp.yaw_acc);


}

void GeometricControl::Run() {
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

    const hrt_abstime now = angular_velocity.timestamp_sample;

    // Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
    // const float dt =
    //     math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
    // _last_run = now;

    // // const Vector3f angular_accel{v_angular_acceleration.xyz};
    // const Vector3f rates{angular_velocity.xyz};

    /* check for updates in other topics */
    update_state();
    update_setpoint();
    _vehicle_status_sub.update(&_vehicle_status);

    // reset integral if disarmed
  
    // run rate controller
    
    const Vector3f e3 (0,0,1);

    const Vector3f ex = (_x - _xd).zero_if_nan();
    const Vector3f ev = (_v - _vd).zero_if_nan();


    //PX4_INFO("ex: %f, %f, %f", (double)ex(0), (double)ex(1), (double)ex(2));

    const Vector3f force = _mass * (-_kx * ex - _kv * ev) + _mass * _g * e3 + _mass * _ad.zero_if_nan();
    const Vector3f b3 = force.unit();
    const Vector3f b2 = b3.cross(_b1d).unit();
    const Vector3f b1 = b2.cross(b3).unit();

    Dcm<float> Rd;
    for (size_t i=0; i < 3 ; i++){
	    Rd(i, 0) = b1(i);
	    Rd(i, 1) = b2(i);
	    Rd(i, 2) = b3(i);
    }

    const Vector3f eR = Vector3f(0.5f * (Rd.T() * _R - _R.T() * Rd).vee() ).zero_if_nan();
    const Vector3f eOmega = (_Omega - _R.T() * Rd * _Omegad).zero_if_nan();

    float f = force.dot(Vector3f(_R * e3));

    PX4_INFO("Re3: %.3f, %.3f, %.3f", (double)(Vector3f(_R*e3)(0)), (double)(Vector3f(_R*e3)(1)), (double)(Vector3f(_R*e3)(2)));
    //PX4_INFO("f: %.3f, force: %.3f, %.3f, %.3f", (double)f, (double)force(0), (double)force(1), (double)force(2));
    
    const Vector3f M = -_kR*eR - _kOmega * eOmega + _Omega.cross(_J * _Omega) - _J * ( _Omega.hat() * _R.T() * Rd * _Omegad - _R.T() * Rd * _alphad);

    // convert f, M to pwm
    Vector4f pwm_cmds = fM_to_pwm(f, 0.0f*M); //TODO:REMOVE
    
    // send pwm commands to motors
    publishPWM(pwm_cmds, now);

  }

  perf_end(_loop_perf);
}


Vector4f GeometricControl::fM_to_pwm(float f, const Vector3f M){


    // determine omega^2
    const Vector4f fM ( f, M(0), M(1), M(2) );
    Vector4f omegas = Vector4f(_invG * fM).signedsqrt();
	
    PX4_INFO("Thrust: %.3f, omegas: %.3f, %.3f, %.3f, %.3f", (double)f, (double)omegas(0), (double)omegas(1), (double)omegas(2), (double)omegas(3));

    // assume RPM is linear with PWM command
    Vector4f pwms = omegas / _omega_max;

    // clamp to range
    for (size_t i=0; i < 3; i++){
	    pwms(i) = std::min(std::max(-1.0f, pwms(i)), 1.0f);
    }

    return pwms;
}

void GeometricControl::publishPWM( const Vector4f &pwm, const hrt_abstime &timestamp_sample){

   // actuator_motors_s msg = {};
   // msg.timestamp = hrt_absolute_time();
   // msg.timestamp_sample = timestamp_sample;

   // msg.reversible_flags = 255; // _param_r_rev.get(); // TODO check!

   // for (size_t i = 0; i < 4; i++) {
   //   msg.control[i] = pwm(i);
   // }

   // _actuator_motors_pub.publish(msg);


  actuator_controls_s msg2;
  msg2.timestamp = hrt_absolute_time();
  msg2.timestamp_sample = timestamp_sample;
  msg2.control[actuator_controls_s::INDEX_ROLL] = pwm(0);
  msg2.control[actuator_controls_s::INDEX_PITCH] = pwm(1);
  msg2.control[actuator_controls_s::INDEX_YAW] = pwm(2);
  msg2.control[actuator_controls_s::INDEX_THROTTLE] = pwm(3);

  _actuator_controls_pub.publish(msg2);

}


// void GeometricControl::publishTorqueSetpoint(
//     const Vector3f &torque_sp, const hrt_abstime &timestamp_sample) {
//   // convert from ENU to NED here
//   vehicle_torque_setpoint_s v_torque_sp = {};
//   v_torque_sp.timestamp = hrt_absolute_time();
//   v_torque_sp.timestamp_sample = timestamp_sample;
//   v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
//   v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
//   v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? -torque_sp(2) : 0.0f;
// 
//   _vehicle_torque_setpoint_pub.publish(v_torque_sp);
// }
// 
// void GeometricControl::publishThrustSetpoint(
//     const float thrust_sp, const hrt_abstime &timestamp_sample) {
//   // convert from ENU to NED here
//   vehicle_thrust_setpoint_s v_thrust_sp = {};
//   v_thrust_sp.timestamp = hrt_absolute_time();
//   v_thrust_sp.timestamp_sample = timestamp_sample;
//   v_thrust_sp.xyz[0] = 0.0f;
//   v_thrust_sp.xyz[1] = 0.0f;
//   v_thrust_sp.xyz[2] =
//       PX4_ISFINITE(_thrust_sp) ? -thrust_sp : 0.0f; // Z is Down
// 
//   _vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
// }

// void GeometricControl::updateActuatorControlsStatus(const actuator_controls_s
// &actuators, float dt)
// {
// 	for (int i = 0; i < 4; i++) {
// 		_control_energy[i] += actuators.control[i] * actuators.control[i] *
// dt;
// 	}
//
// 	_energy_integration_time += dt;
//
// 	if (_energy_integration_time > 500e-3f) {
//
// 		actuator_controls_status_s status;
// 		status.timestamp = actuators.timestamp;
//
// 		for (int i = 0; i < 4; i++) {
// 			status.control_power[i] = _control_energy[i] /
// _energy_integration_time; 			_control_energy[i] = 0.f;
// 		}
//
// 		_actuator_controls_status_0_pub.publish(status);
// 		_energy_integration_time = 0.f;
// 	}
// }

int GeometricControl::task_spawn(int argc, char *argv[]) {

  GeometricControl *instance = new GeometricControl();

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

int GeometricControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int GeometricControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implements a geometric controller to command the quad

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("geometric_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int geometric_control_main(int argc, char *argv[]) {
  return GeometricControl::main(argc, argv);
}
