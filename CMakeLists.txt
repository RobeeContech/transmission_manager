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


include_directories(include)
add_library(${PROJECT_NAME} src/transmission_manager.cpp include/transmission_manager/transmission_manager.hpp)
ament_target_dependencies(${PROJECT_NAME}
  transmission_interface
  hardware_interface
  rclcpp
  pluginlib
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  ament_package()