
add_subdirectory(AttitudeControl)

px4_add_module(
	MODULE modules__indi_att_control
	MAIN indi_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		indi_att_control_main.cpp
		indi_att_control.hpp
	DEPENDS
		AttitudeControl
		mathlib
		px4_work_queue
	)
