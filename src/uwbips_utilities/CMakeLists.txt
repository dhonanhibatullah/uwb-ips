cmake_minimum_required(VERSION 3.8)
project(uwbips_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodemanReq.msg"
  "msg/AnchorData.msg"
  "msg/TagData.msg"
  "msg/SerialData.msg"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(node_manager
  src/node_manager_main.cpp
  src/uwbips_utilities/node_manager.cpp
  src/uwbips_utilities/anchor_node.cpp
  src/uwbips_utilities/tag_node.cpp
)
ament_target_dependencies(node_manager
  rclcpp
)
target_link_libraries(node_manager
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(espnow_serial
  src/espnow_serial_main.cpp
  src/uwbips_utilities/espnow_serial
)
ament_target_dependencies(espnow_serial
  rclcpp
)
target_link_libraries(espnow_serial
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS
  node_manager
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
