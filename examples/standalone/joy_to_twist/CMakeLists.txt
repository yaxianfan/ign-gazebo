cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-transport7 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport7_VERSION_MAJOR})

find_package(sdformat8 REQUIRED)
set(SDF_VER ${sdformat8_VERSION_MAJOR})

add_executable(joy_to_twist joy_to_twist.cc)
target_link_libraries(joy_to_twist
  ignition-transport${IGN_TRANSPORT_VER}::core
  sdformat${SDF_VER}::sdformat${SDF_VER}
)

