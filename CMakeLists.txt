cmake_minimum_required(VERSION 3.8)
project(roboto_diffbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(gui_integration_node src/gui_integration_node.cpp)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(controller_manager REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/diffbot_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/roboto_diffbot>
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}  
)

ament_target_dependencies(
  gui_integration_node PUBLIC
  tf2_ros
  geometry_msgs
  std_srvs
  rclcpp_lifecycle
  lifecycle_msgs
  nav2_msgs
  rclcpp_action
)

target_link_libraries(${PROJECT_NAME} PUBLIC serial)
pluginlib_export_plugin_description_file(hardware_interface roboto_diffbot.xml)

install(
  DIRECTORY launch description config sim hardware
  DESTINATION share/${PROJECT_NAME}
  )
install(TARGETS roboto_diffbot
  EXPORT export_roboto_diffbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  gui_integration_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY hardware/include/
  DESTINATION include/roboto_diffbot
)

# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/description/hooks/${PROJECT_NAME}.dsv.in")
# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/description/hooks/${PROJECT_NAME}.sh.in")
ament_export_targets(export_roboto_diffbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
