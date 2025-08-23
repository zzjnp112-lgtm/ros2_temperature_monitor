cmake_minimum_required(VERSION 3.8)
project(smart_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(using_msgs REQUIRED) # 你的自定义消息包

add_executable(smart_meter_node src/smart_meter_node.cpp)
add_executable(smart_meter_waring src/smart_meter_waring.cpp)

ament_target_dependencies(smart_meter_node
  rclcpp
  std_msgs
  std_srvs
  using_msgs
)

ament_target_dependencies(smart_meter_waring
  rclcpp
  std_msgs
  std_srvs
  using_msgs
)

install(TARGETS
  smart_meter_node
  smart_meter_waring
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

ament_package()