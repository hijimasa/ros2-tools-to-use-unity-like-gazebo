cmake_minimum_required(VERSION 3.16)
project(differential_system_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(differential_system_controller_parameters
  src/differential_system_controller_parameter.yaml
)

add_library(differential_system_controller SHARED
  src/differential_system_controller.cpp
)
target_compile_features(differential_system_controller PUBLIC cxx_std_17)
target_include_directories(differential_system_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include/differential_system_controller>
)
target_link_libraries(differential_system_controller PUBLIC differential_system_controller_parameters)
ament_target_dependencies(differential_system_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(differential_system_controller PRIVATE "differential_system_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface differential_system_plugin.xml)

#if(BUILD_TESTING)
if(False)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_differential_system_controller
    test/test_differential_system_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_differential_system_controller.yaml)
  target_link_libraries(test_differential_system_controller
    differential_system_controller
  )
  ament_target_dependencies(test_differential_system_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(test_load_differential_system_controller
    test/test_load_differential_system_controller.cpp
  )
  target_include_directories(test_load_differential_system_controller PUBLIC
  test)
  ament_target_dependencies(test_load_differential_system_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/differential_system_controller
)
install(TARGETS differential_system_controller differential_system_controller_parameters
  EXPORT export_differential_system_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_differential_system_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
