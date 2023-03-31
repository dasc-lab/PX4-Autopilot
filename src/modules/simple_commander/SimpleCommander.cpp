// Devansh Agrawal 
// March 2023

#include "SimpleCommander.hpp"

SimpleCommander::SimpleCommander() :
	ModuleParams(nullptr)
{

	_boot_timestamp = hrt_absolute_time();
	_last_preflight_check = hrt_absolute_time();
	_last_arm_status_pub = hrt_absolute_time();
	
}

SimpleCommander::~SimpleCommander(){
	//perf_free();
}

void SimpleCommander::run()
{

	while(!should_exit()){

		// check for parameter updates
		const bool params_updated = _parameter_update_sub.updated();
		if (params_updated) {
			parameter_update_s update;
			_parameter_update_sub.copy(&update);
			updateParams();
		}

		if (hrt_elapsed_time(&_last_arm_status_pub) > 500_ms){
			publish_arm_status();
		}


		run_state_machine();

		px4_usleep(10_ms);

	}
}

bool SimpleCommander::check_has_landed(){
	// check if the drone has successfully landed
	return true;
}

bool SimpleCommander::set_state(VehicleState new_state){

	switch(new_state){
		case VehicleState::DISARMED:
			_state = VehicleState::DISARMED;
			publish_arm_status();
			return true;

		case VehicleState::ARMED:

			if (_state != VehicleState::DISARMED){
				PX4_WARN("ARMING DENIED! Vehicle is not currently disarmed.");
				return false;
			}

		  if (!preflight_check()){
			  //PX4_WARN("ARMING DENIED! Vehicle did not pass preflight checks.");
			  return false;
			}

			PX4_INFO("Arming...");
			_state = VehicleState::ARMED;
			publish_arm_status();
			return true;

		case VehicleState::OFFBOARD:
			if (_state != VehicleState::ARMED){
				PX4_WARN("OFFBOARD MODE DENIED! Vehicle is not currently armed.");
				return false;
			}

			// TODO(dev): check if offboard mode messages have been recevied.

			PX4_INFO("Switching to offboard control...");
			_state = VehicleState::OFFBOARD;
			return true;

		case VehicleState::LAND:

			if (_state == VehicleState::DISARMED){
				PX4_WARN("LAND MODE DENIED! Vehicle is currently disarmed");
				return false;
			}

			PX4_INFO("Switching to land mode..."); 
			_state = VehicleState::LAND;
			return true;
		
		default:
			return false;
	}
}

void SimpleCommander::publish_arm_status(){
	// should be published at 2Hz or when arming state changes	
	
	// first set all to false
	// see msg definition to know what these mean
	actuator_armed_s actuator_armed;
  actuator_armed.armed = false;
  actuator_armed.prearmed = false;
  actuator_armed.ready_to_arm = false;
  actuator_armed.lockdown = false;
  actuator_armed.manual_lockdown = false;
  actuator_armed.force_failsafe = false;
  actuator_armed.in_esc_calibration_mode = false;
  actuator_armed.soft_stop = false;

	// vehicle_status
	vehicle_status_s vehicle_status;
	vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_INIT;
	// vehicle_status.system_type = //TODO: MAVTYPE?
	// vehicle_status.system_id = //MAVLINK system_id
	// vehicle_status.component_id;
	//vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	// replace the ones that need to be replaced
	switch (_state){
		case VehicleState::DISARMED:
			actuator_armed.ready_to_arm = true;
			actuator_armed.lockdown = true;
			break;
		case VehicleState::ARMED:
			actuator_armed.armed = true;
			actuator_armed.prearmed = true;
	    vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
			break;
		case VehicleState::OFFBOARD:
			actuator_armed.armed = true;
	    vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
			break;
		case VehicleState::LAND:
			actuator_armed.armed = true;
	    vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
			break;
		default:
			// should never get here
			return;
	}
	
	// publish
	auto now = hrt_absolute_time();
	actuator_armed.timestamp = now;
	vehicle_status.timestamp = now;
  _last_arm_status_pub = now;
	_actuator_armed_pub.publish(actuator_armed);
	_vehicle_status_pub.publish(vehicle_status);

}



void SimpleCommander::run_state_machine(){

	switch (_state){

		case VehicleState::DISARMED:
			set_state(VehicleState::ARMED);
			
			// // check if arming message received
			// bool arming_msg_received =  true;	
			// if (arming_msg_received){
			//     set_state(VehicleState::ARMED);
			// }

			return;

		case VehicleState::ARMED:
			set_state(VehicleState::OFFBOARD);
			
			// // check if offboard message received
			// bool offboard_msg_received =  true;
			// 
			// if (offboard_msg_received){
			//     set_state(VehicleState::OFFBOARD);
			// }

			return;

		case VehicleState::OFFBOARD:

			publish_takeoff_setpoint();

			return;

		case VehicleState::LAND:
			if (check_has_landed()){
				set_state(VehicleState::DISARMED);
			}
			return;

		default:
			// should never get here
			return;
	}

}

void SimpleCommander::publish_takeoff_setpoint(){


	// publish offboard setpoint
	vehicle_local_position_setpoint_s setpoint;
	setpoint.timestamp = hrt_absolute_time();

	setpoint.x = 0.0;
	setpoint.y = 0.0;
	setpoint.z = -4.0;
	setpoint.yaw = 0.0;
	setpoint.yawspeed = 0.0;
	setpoint.vx = 0.0;
	setpoint.vy = 0.0;
	setpoint.vz = 0.0;
	for (int i=0; i < 3; i++){
		setpoint.acceleration[i] = 0.0;
		setpoint.jerk[i] = 0.0;
		setpoint.thrust[i] = 0.0;
	}


	// publish offboard command mode
  vehicle_control_mode_s mode;
	mode.timestamp = hrt_absolute_time();
  mode.flag_armed = true;

	mode.flag_multicopter_position_control_enabled = true;
	mode.flag_control_manual_enabled = false;
	mode.flag_control_auto_enabled = false;	
	mode.flag_control_offboard_enabled = true;
	mode.flag_control_rates_enabled = true;
	mode.flag_control_attitude_enabled = true;
	mode.flag_control_acceleration_enabled = true;
	mode.flag_control_velocity_enabled = true;
	mode.flag_control_position_enabled = true;
	mode.flag_control_altitude_enabled = true;
	mode.flag_control_climb_rate_enabled = true;
	mode.flag_control_termination_enabled = false;


	_vehicle_control_mode_pub.publish(mode);
	_trajectory_setpoint_pub.publish(setpoint);


}



int SimpleCommander::print_status()
{
	PX4_INFO("PRINT STATUS");
	return 0;
}

bool SimpleCommander::preflight_check_ekf(){

	// fail if within first 10 seconds of boot
	// since EKF is likely to be bogus then
	
	auto time_since_boot  = hrt_elapsed_time(&_boot_timestamp);
	if (time_since_boot < 5_s){
		PX4_WARN("EKF waiting 5s since boot.");
		return false;
	}

	// check that EKF has valid position
	if (_vehicle_local_position_sub.updated()){
		vehicle_local_position_s ekf_state; 
		_vehicle_local_position_sub.update(&ekf_state);
		
		if (!ekf_state.xy_valid){
			PX4_WARN("EKF xy not valid");
			return false;
		}

		if (!ekf_state.z_valid){
			PX4_WARN("EKF z not valid");
			return false;
		}

		if (!ekf_state.v_xy_valid){
			PX4_WARN("EKF v_xy not valid");
			return false;
		}

		if (!ekf_state.v_z_valid){
			PX4_WARN("EKF v_z not valid");
			return false;
		}

		if (!ekf_state.heading_good_for_control){
			PX4_WARN("EKF heading not valid");
			return false;
		}
	}

	return true;
}

bool SimpleCommander::preflight_check(){
	
	if (hrt_elapsed_time(&_last_preflight_check) < 250_ms){
		return false;
	}
	_last_preflight_check = hrt_absolute_time();

	// check mag
	// check accel
	// check gyro
	// check baro
	// check IMU consistency 
	// check distance sensor
	// check airspeed sensor
	// check RC calibration
	// check system power

	// check EKF
	if (!preflight_check_ekf()){
		PX4_WARN("EKF preflight check failed");
		return false;
	}

	PX4_INFO("PREFLIGHT PASSED!");
	return true;
}


bool SimpleCommander::handle_command_arm(){
	PX4_INFO("COMMAND ARM");
	return set_state(VehicleState::ARMED);
}

bool SimpleCommander::handle_command_disarm(){

	PX4_INFO("COMMAND DISARM");
	return set_state(VehicleState::DISARMED);
	return true;

}

bool SimpleCommander::handle_command_land(){
	PX4_INFO("COMMAND LAND");
	return set_state(VehicleState::LAND);
}


int SimpleCommander::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	PX4_INFO("CUSTOM COMMAND: %s", argv[0]);

	if (!strcmp(argv[0], "calibrate")) {
    //TODO(dev): (copy from original commander?)
	}

	if (!strcmp(argv[0], "preflight_check")) {
		get_instance()->preflight_check();
		return 0;
	}

	if (!strcmp(argv[0], "arm")) {
    get_instance()-> handle_command_arm();
		return 0;
	}

	if (!strcmp(argv[0], "disarm")) {
    get_instance()->handle_command_disarm();
		return 0;
	}

	if (!strcmp(argv[0], "land")) {
    get_instance()->handle_command_land();
		return 0;
	}

	return print_usage("unknown command");
}


int SimpleCommander::task_spawn(int argc, char* argv[])
{

  _task_id = px4_task_spawn_cmd("simple_commander",
      SCHED_DEFAULT,
      SCHED_PRIORITY_DEFAULT + 40,
      3250,
      (px4_main_t)&run_trampoline,
      (char * const *)argv);

  if (_task_id < 0){
    _task_id = -1;
    return -errno;
  }

  // wait until task is up and running
  if (wait_until_running() < 0){
    _task_id = -1;
    return -1;
  }

  return 0;
}

SimpleCommander *SimpleCommander::instantiate(int argc, char*argv[]){
  SimpleCommander *instance = new SimpleCommander();

  return instance;
}



int SimpleCommander::print_usage(const char * reason)
{
  if (reason){
    PX4_INFO("%s", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
A simplified Commander module
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("simple_commander", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("arm");
		PRINT_MODULE_USAGE_COMMAND("disarm");
    PRINT_MODULE_USAGE_COMMAND("preflight_check");
    PRINT_MODULE_USAGE_COMMAND("land");

    return 1;
}

extern "C" __EXPORT int simple_commander_main(int argc, char*argv[])
{
	return SimpleCommander::main(argc, argv);
}
