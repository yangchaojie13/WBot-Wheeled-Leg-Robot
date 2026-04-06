cmake_minimum_required(VERSION 3.8)
project(wbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 生成可执行文件
add_executable(thigh_pd_controller src/thigh_pd_controller.cpp)
ament_target_dependencies(thigh_pd_controller rclcpp sensor_msgs std_msgs)

add_executable(lqr_controller src/lqr_controller.cpp)
ament_target_dependencies(lqr_controller rclcpp sensor_msgs std_msgs)

# 安装可执行文件，让 ros2 run 能够找到它
install(TARGETS
  thigh_pd_controller
  lqr_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
