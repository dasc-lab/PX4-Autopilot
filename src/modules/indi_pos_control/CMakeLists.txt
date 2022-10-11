
add_subdirectory(PositionControl)
add_subdirectory(Takeoff)

px4_add_module(
	MODULE modules__indi_pos_control
	MAIN indi_pos_control
	COMPILE_FLAGS
	SRCS
    IndiPositionControl.cpp
    IndiPositionControl.hpp
	DEPENDS
		PositionControl
		Takeoff
		controllib
		geo
		SlewRate
	)
