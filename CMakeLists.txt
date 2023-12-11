cmake_minimum_required(VERSION 3.5)
project(ros2_sr04m_sensor)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  /usr/include/wiringpi2
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ros2_sr04m_sensor
)

add_executable(AJ_SR04M_Node src/AJ_SR04M_Node.cpp src/libSonar.cpp)
# target_include_directories(AJ_SR04M_Node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  AJ_SR04M_Node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

target_link_libraries(
  AJ_SR04M_Node 
  wiringPiDev 
  wiringPi
)

link_directories(/usr/lib/aarch64-linux-gnu)

install(TARGETS AJ_SR04M_Node
  DESTINATION lib/${PROJECT_NAME})

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
