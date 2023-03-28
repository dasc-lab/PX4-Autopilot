// Devansh Agrawal 
// March 2023




#include "SimpleCommander.hpp"


SimpleCommander::SimpleCommander() :
	ModuleParams(nullptr)
{
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


		run_state_machine();

	}
}

void SimpleCommander::run_state_machine(){

	switch (_vehicle_state){

		case VehicleState::IDLE:
			return;

		case VehicleState::TAKEOFF:
			// check if the target location is reached
			// if yes, switch to hover mode
		
			return;

		case VehicleState::HOVER:
			// wait for offboard messages
			return;

		case VehicleState::MANUAL_CONTROL:
			return;

		case VehicleState::OFFBOARD_CONTROL:
			// check how long it has been since last offboard message
			// land if necessary
			return;

		case VehicleState::LAND:
			// check if landed
			// switch to IDLE and disarmed if necessary

		default:
			// should never get here
			return;
	}

}



int SimpleCommander::print_status()
{
	PX4_INFO("PRINT STATUS");
	return 0;
}


bool SimpleCommander::set_setpoint_takeoff()
{
	return true;
}


bool SimpleCommander::set_setpoint_land()
{
	return true;
}


bool SimpleCommander::preflight_check(){
	PX4_INFO("RUNNING PREFLIGHT CHECKS");
	if (_arming_state == ArmingState::ARMED)
  	return true;
	if (_arming_state == ArmingState::DISARMED)
  	return true;

	return true;
}


bool SimpleCommander::command_arm(){
	PX4_INFO("COMMAND ARM");

	if (SimpleCommander::preflight_check()) {
		_arming_state = ArmingState::ARMED;

		return true;
	
	} else {
		
		command_disarm();
		return false;
	}

	return false;
}

bool SimpleCommander::command_disarm(){

	PX4_INFO("COMMAND DISARM");
	_arming_state = ArmingState::DISARMED;
	return true;

}

bool SimpleCommander::command_takeoff(){
	PX4_INFO("COMMAND TAKEOFF");

	// check that it is not already flying
	if (_vehicle_state != VehicleState::IDLE){
		PX4_WARN("Vehicle is not idle! Ignoring takeoff request!");
	}

	// try arming
	if (_arming_state != ArmingState::ARMED){
	   command_arm();
	}

	// check that vehicle is armed
	if (_arming_state != ArmingState::ARMED){
		PX4_WARN("UNABLE TO ARM! Ignorning takeoff request");
		return false;
	}

	// publish takeoff setpoint
	set_setpoint_takeoff();
	_vehicle_state = VehicleState::TAKEOFF;
	return true;

}

bool SimpleCommander::command_land(){
	PX4_INFO("COMMAND LAND");

	if (_arming_state != ArmingState::ARMED) {
		PX4_WARN("NOT ARMED! Ignoring land request");
		return false;
	}

	set_setpoint_land();
	return true;

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

	if (!strcmp(argv[0], "check")) {
		get_instance()->preflight_check();
	}

	if (!strcmp(argv[0], "arm")) {
    get_instance()-> command_arm();
		return 0;
	}

	if (!strcmp(argv[0], "disarm")) {
    get_instance()->command_disarm();
	}

	if (!strcmp(argv[0], "takeoff")) {
    get_instance()->command_takeoff();
	}

	if (!strcmp(argv[0], "land")) {
    get_instance()->command_land();
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
    PRINT_MODULE_USAGE_COMMAND("preflight_check");
    PRINT_MODULE_USAGE_COMMAND("takeoff");
    PRINT_MODULE_USAGE_COMMAND("land");

    return 1;
}

extern "C" __EXPORT int simple_commander_main(int argc, char*argv[])
{
	return SimpleCommander::main(argc, argv);
}
