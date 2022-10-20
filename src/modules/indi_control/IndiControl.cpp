// Indi Controller
// Devansh Agrawal Oct 2022


#include "IndiControl.hpp"

using namespace matrix;

IndiControl::IndiControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_a_filter(RATE_ACCEL, RATE_LPF),
  _tau_bz_filter(RATE_TAU_BZ, RATE_LPF),
  _ang_vel_filter(RATE_ANG_VEL, RATE_LPF),
  _torque_filter(RATE_TORQUE, RATE_LPF)
{

	// set default setpoint
	_setpoint.position[0] = 0.0;
	_setpoint.position[1] = 0.0;
	_setpoint.position[2] = -0.25;
	_setpoint.yaw = 0.0;

}

IndiControl::~IndiControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool IndiControl::init()
{

	update_parameters();

	// Run on fixed interval
  uint32_t interval = (uint32_t)(1.0e6/((double)RATE_TORQUE));
	ScheduleOnInterval(interval); // 500 us interval, 2000 Hz rate // TODO: Make this a parameter

	return true;
}


void IndiControl::update_parameters()
{

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

  Vector4f motor_pos_x ( lx, -lx, lx, -lx);
  Vector4f motor_pos_y (ly, -ly, -ly, ly);
  Vector4f motor_dir (-1, -1, 1, 1); // positive for torque in +bz direction (body FRD frame)

  _G1.zero();
  for (size_t i=0; i<4; i++){
    // torque about body x:
    _G1(0, i) = motor_pos_y(i) * _thrust_constant;
    // torque about body y:
    _G1(1, i) = motor_pos_x(i) * _thrust_constant;
    // torque about body z:
    _G1(2, i) = motor_dir(i) * _torque_constant;
    // thrust in body z:
    _G1(3, i) = - _thrust_constant;
  }
  _inv_G1 = _G1.I(); // gets the inverse

}


void IndiControl::service_subscriptions()
{

	cb_params();
	cb_vehicle_status();
	cb_vehicle_local_position();
	cb_vehicle_attitude();
	cb_vehicle_angular_velocity();
	cb_trajectory_setpoint();

}

void IndiControl::compute_cmd_accel()
{
	Vector3f k_pos(18, 18, 13.5);
	Vector3f k_vel(7.8, 7.8, 5.9);
	Vector3f k_acc(0.5, 0.5, 0.3);

  k_pos *= 0.1;
  k_vel *= 0.1;
  k_acc *= 0.1;

	Vector3f x(_local_position.x,  _local_position.y,  _local_position.z);
	Vector3f v(_local_position.vx, _local_position.vy, _local_position.vz);
	Vector3f a(_local_position.ax, _local_position.ay, _local_position.az);

	Vector3f x_ref(_setpoint.position);
	Vector3f v_ref(_setpoint.velocity);
	Vector3f a_ref(_setpoint.acceleration);

	// acceleration command (EZRA, EQ:17)
	_a_cmd = a_ref.zero_if_nan()
		 + k_pos.emult((x_ref - x).zero_if_nan())
		 + k_vel.emult((v_ref - v).zero_if_nan())
		 + k_acc.emult((a_ref - _a_filt).zero_if_nan());
 
 if (_armed){
  PX4_INFO("POS: %f, %f, %f", (double)x(0), (double)x(1), (double) x(2)); 
   // PX4_INFO("Z: %f, _a_cmd: %f, a: %f", (double)_local_position.z, (double)_a_cmd(2), (double)a(2));
 }
}

void IndiControl::compute_cmd_thrust()
{
	// EZRA, EQ:20
	_tau_bz_cmd = _tau_bz_filt + _a_cmd - _a_filt;

	// EZRA, EQ:21
	_thrust_cmd = - _mass * _tau_bz_cmd.norm();

  Vector3f tau_bz = (_thrust_cmd / _mass) * _bz;
  _tau_bz_filt = _tau_bz_filter.apply(tau_bz);

	//publish_thrust_cmd();
}


void IndiControl::compute_cmd_quaternion()
{
	// get body to inertial quaterion
	Quatf xi(_attitude.q);

	// EZRA, EQ:22
	Vector3f neg_bz_cmd_body = xi.rotateVectorInverse(- _tau_bz_cmd);

	// EZRA, EQ:23
	Vector3f iz(0, 0, 1);
	Quatf xi_bar(iz, neg_bz_cmd_body);  // min rotation constructor (defined in matrix/Quaternion.hpp)

	// EZRA, EQ:24
	const float yaw_ref = _setpoint.yaw;
	Vector3f heading_vec_inertial(std::sin(yaw_ref), -std::cos(yaw_ref), 0);
	Quatf xi_xi_bar = xi * xi_bar;
	Vector3f heading_vec_bar = xi_xi_bar.rotateVectorInverse(heading_vec_inertial);

	// EZRA, EQ:25
	Quatf xi_yaw(1, 0, 0, -heading_vec_bar(0) / heading_vec_bar(1));
	xi_yaw.normalize();

	// EZRA, EQ:26
	_xi_cmd = xi_bar * xi_yaw;

	publish_xi_cmd();

}

void IndiControl::compute_cmd_ang_accel()
{

	// EZRA, EQ:27
	float xi_w = _xi_cmd(0);
	Vector3f xi_e = (2.0f * std::acos(xi_w) / std::sqrt(1.0f - xi_w * xi_w)) * _xi_cmd.imag();

	Vector3f k_xi(175, 175, 82);
	Vector3f k_omega(19.5, 19.5, 19.2);

  k_xi *= 0.01;
  k_omega *= 0.01;

	Vector3f ang_vel_ref(0, 0, 0); //TODO: FIX
	Vector3f ang_accel_ref(0, 0, 0); // TODO: FIX

	// EZRA, EQ:28
	_ang_accel_cmd = ang_accel_ref.zero_if_nan()
			 + k_xi.emult(xi_e.zero_if_nan())
			 + k_omega.emult((ang_vel_ref - _ang_vel_filt).zero_if_nan());    // why do i need the filtered version?

	publish_ang_accel_cmd();

}

void IndiControl::compute_cmd_torque()
{

	// EZRA, EQ:31
	_torque_cmd = _torque_filt.zero_if_nan()
		      + _J * ((_ang_accel_cmd - _ang_accel_filt).zero_if_nan());


	_torque_filt = _torque_filter.apply(_torque_cmd);
	
  //publish_torque_cmd();

}

void IndiControl::compute_cmd_pwm()
{

  //_torque_cmd(2) = 0.0;

  // note RPM is actually in units of radians per second

  // translate from thrust and torque setpoints to motor RPM
  Vector4f mu_T ( _torque_cmd(0), _torque_cmd(1), _torque_cmd(2), _thrust_cmd );
  Vector4f rpm_sq = _inv_G1 * mu_T;

  // translate from motor RPM to motor PWM
  Vector4f rpm = rpm_sq.sqrt();

  for (size_t i=0; i<4; i++){
    _pwm_cmd(i) = rpm(i) / _max_motor_rpm;
    _pwm_cmd(i) = math::constrain(_pwm_cmd(i), -1.0f, 1.0f);
    if (_armed) {
      //PX4_INFO("Z: %f, MOT%zu, RPM: %f, PWM: %f", (double)_local_position.z, i, (double)rpm(i), (double)_pwm_cmd(i));
    }
  }

  publish_pwm_cmd();

}




void IndiControl::compute_cmd_ang_accel_geometric()
{

  const Vector3f kx (4.0, 4.0, 4.0);
  const Vector3f kv (2.0, 2.0, 2.0);
  const Vector3f kR (0.35, 0.35, 0.35);
  const Vector3f kOmega (0.1, 0.1, 0.1);

  // construct the required acceleration vector
  Vector3f _z (0,0,1);
  const float g = 9.81;

	Vector3f x(_local_position.x,  _local_position.y,  _local_position.z);
	Vector3f v(_local_position.vx, _local_position.vy, _local_position.vz);

	Vector3f x_ref(_setpoint.position);
	Vector3f v_ref(_setpoint.velocity);


  Vector3f acc = -g*_z 
    - kx.emult( (x - x_ref).zero_if_nan())
    - kv.emult( (v - v_ref).zero_if_nan());

  Vector3f b3d = (acc.norm() < 0.01f) ? _z : -acc.unit();

  float yaw_ref = 0.0;
  float b1dx = cosf(yaw_ref);
  float b1dy = sinf(yaw_ref);
  float b1dz = 0.0f;

  const Vector3f b1d(b1dx, b1dy, b1dz);
  const Vector3f b2d = b3d.cross(b1d).unit();
  const Vector3f b1d_new = b2d.cross(b3d).unit();

  // construct desired rot matrix
  Dcm<float> rotDes;
  for (size_t i=0; i<3; i++){
    rotDes(i,0) = b1d_new(i);
    rotDes(i,1) = b2d(i);
    rotDes(i,2) = b3d(i);
  }

  // construct current rot matrix
  Quatf ang_att (_attitude.q);
  Dcm<float> rotMat(ang_att); 

  // get rotation error
  Vector3f rotMat_err = 0.5f * ((Dcm<float>)((rotDes.T() * rotMat - rotMat.T() * rotDes))).vee();

  // error in angular rate
  Vector3f ang_rate(_ang_vel);
  Vector3f ang_rate_sp(0,0,0);
  Vector3f ang_rate_err = ang_rate - rotMat.T() * rotDes * ang_rate_sp;
  
  // total acceleration desired
  float coll_acc = -acc.dot(rotMat * _z);

  // desired moments
  Vector3f omega_cross_Jomega = ang_rate.cross(_J * ang_rate);

  Vector3f moments = -kR.emult(rotMat_err) - kOmega.emult(ang_rate_err)
    + omega_cross_Jomega
    - _J * (ang_rate.hat() * rotMat.T() * rotDes * ang_rate_sp);

  _ang_accel_cmd = _J.I() * (moments - omega_cross_Jomega);



  publish_setpoints(-coll_acc, _ang_accel_cmd);

}

void IndiControl::publish_setpoints(const float acc, const Vector3f ang_accel){

    vehicle_thrust_setpoint_s msgT;
    msgT.timestamp = hrt_absolute_time();
    msgT.timestamp_sample = _now;
    msgT.xyz[0] = 0.0f;
    msgT.xyz[1] = 0.0f;
    msgT.xyz[2] = (_armed) ? math::constrain( (acc / 9.81f * 0.75f), -1.0f, 1.0f) : 0.0f;

    
    
    _vehicle_thrust_setpoint_pub.publish(msgT);
  


    vehicle_torque_setpoint_s msgM{};
    msgM.timestamp = hrt_absolute_time();
    msgM.timestamp_sample = _now;

    for (size_t i = 0; i < 3; i++) {
      msgM.xyz[i] =  math::constrain(ang_accel(i) / 35.0f, -1.0f, 1.0f);
    }

    _vehicle_torque_setpoint_pub.publish(msgM);
  
  if (_armed){
    PX4_INFO("PUBLISHING: %f, %f, %f, %f :: %f, %f, %f, %f", (double)acc, (double)ang_accel(0), (double)ang_accel(1), (double)ang_accel(2), (double)msgT.xyz[2], (double)msgM.xyz[0],(double)msgM.xyz[1],(double)msgM.xyz[2]);
  }

}


void IndiControl::compute_cmd()
{

	// run the main controller
	// compute_cmd_accel();
	// compute_cmd_thrust();
	// compute_cmd_quaternion();
	// compute_cmd_ang_accel();
	// compute_cmd_torque();
  // compute_cmd_pwm();
  
  compute_cmd_ang_accel_geometric();
}

void IndiControl::construct_setpoint()
{
}


void IndiControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	_now = hrt_abstime();
	service_subscriptions();
	construct_setpoint();
	compute_cmd();

	perf_end(_loop_perf);
}

int IndiControl::task_spawn(int argc, char *argv[])
{
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

int IndiControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int IndiControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IndiControl::print_usage(const char *reason)
{
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

extern "C" __EXPORT int indi_control_main(int argc, char *argv[])
{
	return IndiControl::main(argc, argv);
}
