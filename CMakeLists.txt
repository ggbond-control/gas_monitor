cmake_minimum_required(VERSION 3.8)
project(gas_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(gas_monitor_common STATIC
  src/pump_protocol.cpp
)
target_include_directories(gas_monitor_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(gas_monitor_pump_agent_lib STATIC
  src/pump_agent.cpp
)
target_include_directories(gas_monitor_pump_agent_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gas_monitor_pump_agent_lib PUBLIC gas_monitor_common)
ament_target_dependencies(gas_monitor_pump_agent_lib PUBLIC rclcpp)

add_executable(serial_gas_node src/serial_gas_node.cpp)
target_include_directories(serial_gas_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(serial_gas_node gas_monitor_common)
ament_target_dependencies(serial_gas_node rclcpp diagnostic_msgs std_srvs)

add_executable(gas_monitor_pump_agent src/pump_agent_main.cpp)
target_include_directories(gas_monitor_pump_agent PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gas_monitor_pump_agent gas_monitor_pump_agent_lib)
ament_target_dependencies(gas_monitor_pump_agent rclcpp)

install(TARGETS
  serial_gas_node
  gas_monitor_pump_agent
  gas_monitor_common
  gas_monitor_pump_agent_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch systemd scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
