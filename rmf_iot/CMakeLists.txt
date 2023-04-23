cmake_minimum_required(VERSION 3.8)
project(rmf_iot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_iot_msg REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(rmf_button 
                src/rmf_button.cpp
                src/serial_port.cpp)
target_include_directories(rmf_button PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rmf_button PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(rmf_button rclcpp rmf_iot_msg rmf_dispenser_msgs rmf_ingestor_msgs rmf_task_msgs)
install(TARGETS rmf_button
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY 
     launch
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
