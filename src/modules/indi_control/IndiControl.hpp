// Indi Controller
// Devansh Agrawal Oct 2022


#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/actuator_motors.h>


#include <lib/allocator_qp/include/osqp.h>
#include <lib/allocator_qp/include/workspace.h>

using namespace time_literals;
using namespace matrix;

class IndiControl : public ModuleBase<IndiControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	IndiControl();
	~IndiControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	hrt_abstime _now;
	hrt_abstime _start;

	void service_subscriptions();
	void cb_params();
	void cb_vehicle_status();
	void cb_vehicle_local_position();
	void cb_vehicle_attitude();
	void cb_vehicle_angular_velocity();
	void cb_trajectory_setpoint();

	void update_parameters();

	void construct_setpoint();

	void compute_cmd();
	void compute_cmd_accel();
	void compute_cmd_thrust();
	void compute_cmd_quaternion();
	void compute_cmd_ang_accel();
	void compute_cmd_torque();
  void compute_cmd_pwm();
  void compute_cmd_ang_accel_geometric();

  void update_allocator_rpm_bounds(const float min, const float max);
  void update_allocator_matrices(const Matrix4f H, const Matrix4f G);
  Vector4f solve_allocator_qp(const Vector4f mu_ref);

	void publish_thrust_cmd();
	void publish_xi_cmd();
	void publish_ang_accel_cmd();
	void publish_torque_cmd();
  void publish_pwm_cmd();
  void publish_setpoints(const float acc, const Vector3f ang_accel);

  // Quad_params
  float _mass;
	Matrix3f _J;
  float _max_rpm; // used for RPM -> PWM conversion
	float _thrust_constant; // Newtons -> PWM normalization factor
  float _torque_constant;
  float _max_motor_rpm = 1.100; // in kilo-rad/s
  Matrix4f _G;
  Matrix4f _H; 


	// Private Variables
	Vector3f _a_cmd;
	Vector3f _a_filt;
	Vector3f _bz;
	Vector3f _tau_bz_cmd;
	Vector3f _tau_bz_filt;
	float _thrust_cmd;
	Quatf _xi_cmd;
  Vector3f _ang_vel;
	Vector3f _ang_vel_filt;
	Vector3f _ang_accel_cmd;
	Vector3f _ang_accel_filt;
	Vector3f _torque_filt;
	Vector3f _torque_cmd;
  Vector4f _rpm_cmd;
  Vector4f _pwm_cmd;


  math::LowPassFilter2p<Vector3f> _a_filter; 
  math::LowPassFilter2p<Vector3f> _tau_bz_filter; 
  math::LowPassFilter2p<Vector3f> _ang_vel_filter;
  math::LowPassFilter2p<Vector3f> _torque_filter;

  // todo: make these into parameters (or grab them from the respective modules)
  	const float RATE_LPF = 30.0; // desired cutoff frequency for low-pass filter
  const float RATE_TORQUE = 800.0;
   const float RATE_ACCEL = 250.0; // frequency of new acceleration messages
  	const float RATE_TAU_BZ = RATE_TORQUE;
  	const float RATE_ANG_VEL = 250.0;


	vehicle_status_s _vehicle_status;
	vehicle_local_position_s _local_position;
	vehicle_attitude_s _attitude;
	vehicle_angular_velocity_s _angular_velocity;
	trajectory_setpoint_s _setpoint;

	// Publications
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
  uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	// Subscriptions
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		//(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		//(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
		(ParamFloat<px4::params::INDI_QUAD_MASS>) _param_mass,
		(ParamFloat<px4::params::INDI_THRUST_K>) _param_thrust_constant,
		(ParamFloat<px4::params::INDI_TORQUE_K>) _param_torque_constant,
		(ParamFloat<px4::params::INDI_JXX>) _param_Jxx,
		(ParamFloat<px4::params::INDI_JYY>) _param_Jyy,
		(ParamFloat<px4::params::INDI_JZZ>) _param_Jzz,
		(ParamFloat<px4::params::INDI_JXY>) _param_Jxy,
		(ParamFloat<px4::params::INDI_JXZ>) _param_Jxz,
		(ParamFloat<px4::params::INDI_JYZ>) _param_Jyz
	)


	bool _armed{false};
};
