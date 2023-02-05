
#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <lib/diffflat/DiffFlatQuad.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_motors.h>
//#include <uORB/topics/actuator_controls_status.h>
//#include <uORB/topics/battery_status.h>
//#include <uORB/topics/control_allocator_status.h>
//#include <uORB/topics/landing_gear.h>
//#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/diffflat_setpoint.h>
//#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
//#include <uORB/topics/vehicle_control_mode.h>
//#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;
using namespace matrix;

class GeometricControl : public ModuleBase<GeometricControl>,
                         public ModuleParams,
                         public px4::WorkItem {
public:
  GeometricControl();
  ~GeometricControl() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  bool init();

private:
  void Run() override;

  /**
   * initialize some vectors/matrices from parameters
   */
  void parameters_updated();

  // void updateActuatorControlsStatus(const actuator_controls_s &actuators,
  // float dt);

  // void publishTorqueSetpoint(const matrix::Vector3f &torque_sp,
  //                            const hrt_abstime &timestamp_sample);
  // void publishThrustSetpoint(const float thrust,
  //                            const hrt_abstime &timestamp_sample);


  void update_state();
  void update_setpoint();

  Vector4f fM_to_pwm(const float f, const Vector3f M);

  void publishPWM( const Vector4f &pwm, const hrt_abstime &timestamp_sample);

  void set_gains(float kx, float kv, float kR, float kOmega)
  {
  
  	_kx = kx;
  	_kv = kv;
  	_kR = kR;
  	_kOmega = kOmega;
  
  }
  
  void set_inertia(float Jxx, float Jyy, float Jzz, float Jxy, float Jxz, float Jyz)
  {
  	_J(0, 0) = Jxx;
  	_J(1, 1) = Jyy;
  	_J(2, 2) = Jzz;
  
  	_J(0, 2) = Jxz;
  	_J(2, 0) = Jxz;
  
  	_J(1, 2) = Jyz;
  	_J(2, 1) = Jyz;
  
  	_J(0, 1) = Jxy;
  	_J(1, 0) = Jxy;
  }

  uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
  uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
  uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
  uORB::Subscription _diffflat_setpoint_sub{ORB_ID(diffflat_setpoint)};

  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};

  uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{
      this, ORB_ID(vehicle_angular_velocity)};

  uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
  uORB::Publication<actuator_controls_s> _actuator_controls_pub{ORB_ID(actuator_controls_0)};

  orb_advert_t _mavlink_log_pub{nullptr};

  perf_counter_t _loop_perf; /**< loop duration performance counter */

  // STATE VARS
  vehicle_status_s _vehicle_status{};
  Vector3f _x, _v, _a, _Omega;
  Dcm<float> _R;
  
  // SETPOINT VARS
  Vector3f _xd, _vd, _ad, _b1d, _Omegad, _alphad;

  // CONTROLLER PARAMS
  float _mass;
  float _kx, _kv, _kR, _kOmega;
  Matrix3f _J;
  hrt_abstime _last_run{0};
  float _k_f, _k_mu, _omega_max;
  Matrix4f _invG;
  
  // CONSTANTS
  Dcm<float> R_ENU_TO_FRD; // will be initialized on construction
  const float _g = 9.81f;

  DEFINE_PARAMETERS((ParamFloat<px4::params::GEO_KX>)_param_geo_kx,
                    (ParamFloat<px4::params::GEO_KV>)_param_geo_kv,
                    (ParamFloat<px4::params::GEO_KR>)_param_geo_kR,
                    (ParamFloat<px4::params::GEO_KOMEGA>)_param_geo_kOmega,
                    (ParamFloat<px4::params::GEO_M>)_param_geo_mass,
                    (ParamFloat<px4::params::GEO_JXX>)_param_geo_Jxx,
                    (ParamFloat<px4::params::GEO_JYY>)_param_geo_Jyy,
                    (ParamFloat<px4::params::GEO_JZZ>)_param_geo_Jzz,
                    (ParamFloat<px4::params::GEO_M1X>)_param_geo_M1x,
                    (ParamFloat<px4::params::GEO_M2X>)_param_geo_M2x,
                    (ParamFloat<px4::params::GEO_M3X>)_param_geo_M3x,
                    (ParamFloat<px4::params::GEO_M4X>)_param_geo_M4x,
                    (ParamFloat<px4::params::GEO_M1Y>)_param_geo_M1y,
                    (ParamFloat<px4::params::GEO_M2Y>)_param_geo_M2y,
		    (ParamFloat<px4::params::GEO_M3Y>)_param_geo_M3y,
                    (ParamFloat<px4::params::GEO_M4Y>)_param_geo_M4y,
                    (ParamFloat<px4::params::GEO_K_F>)_param_geo_k_f,
                    (ParamFloat<px4::params::GEO_K_MU>)_param_geo_k_mu,
                    (ParamFloat<px4::params::GEO_OMEGA_MAX>)_param_geo_omega_max
	  )
};
