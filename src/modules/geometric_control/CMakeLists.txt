
px4_add_module(
	MODULE modules__geometric_control
	MAIN geometric_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		GeometricControl.cpp
		GeometricControl.hpp
	DEPENDS
		mathlib
		px4_work_queue
	)
