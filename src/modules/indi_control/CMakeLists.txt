# Indi Controller
# Devansh Agrawal Oct 2022

# add_subdirectory(allocator_qp)

px4_add_module(
  MODULE modules__indi_control
  MAIN indi_control
  COMPILE_FLAGS
    -O3
  SRCS
    IndiControl.cpp
    IndiControlPubs.cpp
    IndiControlCbs.cpp
    IndiControlAllocator.cpp
    IndiControl.hpp
  DEPENDS
    px4_work_queue
    mathlib
    allocator_qp
)

