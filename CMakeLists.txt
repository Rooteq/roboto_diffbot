cmake_minimum_required(VERSION 3.8)
project(roboto_diffbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(controller_manager REQUIRED)
find_package(ament_cmake REQUIRED)
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
  
install(
  DIRECTORY hardware/include/
  DESTINATION include/roboto_diffbot
)

# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/description/hooks/${PROJECT_NAME}.dsv.in")
# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/description/hooks/${PROJECT_NAME}.sh.in")
ament_export_targets(export_roboto_diffbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
