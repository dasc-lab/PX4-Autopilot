#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>


// Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>

// Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>


using namespace time_literals;

class SimpleCommander : public ModuleBase<SimpleCommander>, public ModuleParams
{
  public:
    SimpleCommander();
    ~SimpleCommander();

    static int task_spawn(int argc, char* argv[]);
    static SimpleCommander *instantiate(int argc, char* argv[]);
    static int custom_command(int argc, char* argv[]);
    static int print_usage(const char * reason = nullptr);

    void run() override;

    int print_status() override;

  private:

    bool preflight_check();

    bool command_arm();
    bool command_disarm();
    bool command_takeoff();
    bool command_land();
    bool set_setpoint_takeoff();
    bool set_setpoint_land();

    void run_state_machine();

    // Publishers
    //
    //
    // Subscribers
	  uORB::SubscriptionInterval				_parameter_update_sub{ORB_ID(parameter_update), 1_s};
    //
    //
    enum class ArmingState {
      DISARMED = 0,
      ARMED
    };

    enum class VehicleState {
      IDLE = 0,          // motors not spinning
      TAKEOFF,           // ignore RC/offboard msgs, publishes a setpoint for takeoff
      HOVER,             // hover at fixed location
      MANUAL_CONTROL,    // controlled by RC
      OFFBOARD_CONTROL,  // controlled by listening to vehicle_offboard_control msgs
      LAND,              // ignore RC/offboard messages, publishes a setpoint
    };

    ArmingState _arming_state = ArmingState::DISARMED;
    VehicleState _vehicle_state = VehicleState::IDLE;
};
