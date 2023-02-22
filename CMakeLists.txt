cmake_minimum_required(VERSION 3.8)
project(transmission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)


add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(transmission_interface REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pluginlib REQUIRED)

  ament_add_gmock(
    test_transmission_manager
    test/test_transmission_manager.cpp
  )
  target_include_directories(test_transmission_manager PRIVATE include)
  ament_target_dependencies(test_transmission_manager
    transmission_interface
    hardware_interface
    rclcpp
    pluginlib
  )
endif()

ament_export_targets("export_${PROJECT_NAME}")

ament_package()