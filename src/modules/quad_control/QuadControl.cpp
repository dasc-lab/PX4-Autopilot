#include "QuadControl.hpp"

using namespace matrix;

QuadControl::QuadControl():
  ModuleParams(nullptr),
  WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{

  // initializer
  parameters_update();
  
}

QuadControl::~QuadControl()
{
  perf_free(_cycle_perf);
}


bool QuadControl::init()
{
  if (!_ang_vel_sub.registerCallback()){
    PX4_ERR("callback registration failed");
    return false;
  }

  _timestamp_last_loop = hrt_absolute_time();
  ScheduleNow();

  return true;
}

void QuadControl::parameters_update()
{
  // check for parameter updates
  if (_parameter_update_sub.updated()){
    parameter_update_s pupdate;
    _parameter_update_sub.copy(&pupdate);

    ModuleParams::updateParams();
  }

}

void QuadControl::Run()
{

  if (should_exit()) {
    _ang_vel_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_cycle_perf);

  // do things
  if (_ang_vel_sub.updated()){

    // check arming
    if (_vehicle_status_sub.update(&_vehicle_status)){
      _armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    }

    if (_armed){

      // grab states
      if (_ang_vel_sub.update(&_state_ang_vel)){
        _controller.update_state_Omega(_state_ang_vel);
        _init_state_Omega = true;
      }

      if (_local_pos_sub.update(&_state_pos)){
        _controller.update_state_pos(_state_pos);
        _init_state_pos = true;
      }

      if (_att_sub.update(&_state_att)){
        _controller.update_state_attitude(_state_att);
        _init_state_att = true;
      }

      // grab setpoint
      if (_trajectory_setpoint_sub.update(&_setpoint)){
        _controller.update_setpoint(_setpoint);
        _init_setpoint = true;
      }

      _initialized = _init_state_Omega && _init_state_pos && _init_state_att && _init_setpoint;

    }

    if (_armed && !_initialized){
      PX4_WARN("ARMED not INITIALIZED");
    }

    if (_armed && _initialized){

      // run controller
      _controller.run();

      // get thrust and torque command
      float thrust_cmd = _controller.get_thrust_cmd();
      thrust_cmd = (thrust_cmd < 0) ? 0.0 : thrust_cmd; // prevent it from trying to be negative;
      Vector3f torque_cmd = _controller.get_torque_cmd().zero_if_nan();

      PX4_INFO("Thrust_cmd: %f, Torque_cmd: %f", (double)thrust_cmd, (double)torque_cmd.norm());

      // do the mixing
      Vector4f pwm_cmd = _mixer.mix(thrust_cmd, torque_cmd);
      
      // publish
      publish_cmd(pwm_cmd);

    }

  }

  perf_end(_cycle_perf);
}

void QuadControl::publish_cmd(Vector4f pwm_cmd)
{

  actuator_outputs_s msg;
  msg.timestamp = hrt_absolute_time();
  msg.noutputs = 4;
  for (size_t i=0; i<4; i++){
    msg.output[i] = pwm_cmd(i);
  }

  _actuator_outputs_pub.publish(msg);

}


int QuadControl::task_spawn(int argc, char* argv[])
{

  QuadControl *instance = new QuadControl();

  if (instance){
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()){
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

int QuadControl::custom_command(int argc, char* argv[])
{
  return print_usage("unknown command");
}

int QuadControl::print_usage(const char * reason){

  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Quadrotor controller
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("quad_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;

}

extern "C" __EXPORT int quad_control_main(int argc, char* argv[])
{
  return QuadControl::main(argc, argv);
}





























