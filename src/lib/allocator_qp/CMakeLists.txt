

message(STATUS "*****************")
message(STATUS "*****************")
message(STATUS "COMPILING OSQP")

# Detect operating system
# ----------------------------------------------
message(STATUS "We are on a ${CMAKE_SYSTEM_NAME} system")
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(IS_LINUX ON)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    set(IS_MAC ON)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(IS_WINDOWS ON)
endif()


set(EMBEDDED 2)
set(PRINTING OFF)
set(PROFILING OFF)
option(DFLOAT "Use floats instead of doubles" OFF)
option(DLONG "Use long (64bit) for indexing" OFF) # by default its ON


# set types for QDLDL
if(DFLOAT)
  set(QDLDL_FLOAT_TYPE "float")
else()
  set(QDLDL_FLOAT_TYPE "double")
endif()

if(DLONG)
  set(QDLDL_INT_TYPE "long long")
else()
  set(QDLDL_INT_TYPE "int")
endif()


set(QDLDL_BOOL_TYPE "unsigned char")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure/qdldl_types.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/include/qdldl_types.h
               NEWLINE_STYLE LF)

# Include math library if EMBEDDED != 1
if(NOT (EMBEDDED EQUAL 1))
    set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lm")
endif()
# Include real time library in linux
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lrt")
endif()

# Generate header file with the global options
# ---------------------------------------------
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure/osqp_configure.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/include/osqp_configure.h
               NEWLINE_STYLE LF)

# Include header directory
# ----------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Set sources
# ----------------------------------------------
add_subdirectory (src/osqp)
add_subdirectory (include)

# Append the generated workspace files and qdldl files
list (APPEND
      osqp_src
      ${CMAKE_CURRENT_SOURCE_DIR}/src/osqp/workspace.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/osqp/qdldl.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/osqp/qdldl_interface.c
)

list (APPEND
      osqp_headers
      ${CMAKE_CURRENT_SOURCE_DIR}/include/workspace.h
      ${CMAKE_CURRENT_SOURCE_DIR}/include/qdldl.h
      ${CMAKE_CURRENT_SOURCE_DIR}/include/qdldl_types.h
      ${CMAKE_CURRENT_SOURCE_DIR}/include/qdldl_interface.h
)



px4_add_library(allocator_qp 
	${osqp_src}
	${osqp_headers})


message(STATUS "DONE COMPILING OSQP")
message(STATUS "*****************")
message(STATUS "*****************")

# set(SRCS
# 	conversions.c
# 	conversions.h
# 	crc.c
# 	crc.h
# 	mavlink_log.cpp
# 	mavlink_log.h
# )
# 
# if(${PX4_PLATFORM} STREQUAL "nuttx")
# 	list(APPEND SRCS
# 		otp.c
# 	)
# endif()
# 
# px4_add_library(systemlib ${SRCS})
# target_compile_options(systemlib PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
