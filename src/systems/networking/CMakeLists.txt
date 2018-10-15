set (network_system_sources
  NetworkPrimary.cc
  NetworkSecondary.cc
)

ign_add_component(network-system
  SOURCES ${network_system_sources}
  GET_TARGET_NAME network_system_target
  CXX_STANDARD 17)

target_link_libraries(${network_system_target}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(
  TARGETS ${network_system_target}
  DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR}
)
