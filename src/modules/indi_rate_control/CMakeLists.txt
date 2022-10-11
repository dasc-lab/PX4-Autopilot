add_subdirectory(GeometricControl)

px4_add_module(
	MODULE modules__indi_rate_control
	MAIN indi_rate_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		IndiRateControl.cpp
		IndiRateControl.hpp
	DEPENDS
		circuit_breaker
		mathlib
		GeometricControl
		RateControl
		px4_work_queue
	)
