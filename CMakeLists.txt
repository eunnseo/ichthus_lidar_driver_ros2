cmake_minimum_required(VERSION 3.5)
project(ichthus_lidar_driver_ros2)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED) # component
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

### Message ###
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Packet.msg"
  DEPENDENCIES builtin_interfaces
)
###############

### Frontend ###
ament_auto_add_library(frontend SHARED
  src/lib/frontend.cpp
  src/net/packet_utils.cpp
  src/sensor/os1_64/os1_64_packet.cpp
  src/sensor/os1_64/os1_64.cpp
)

ament_auto_add_library(frontend_node SHARED
  src/nodes/frontend_node.cpp
)

target_link_libraries(frontend_node
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES} # pcl
  frontend
)

rosidl_target_interfaces(frontend_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_nodes(frontend_node
  PLUGIN "ichthus_lidar_driver_ros2::frontend_node::FrontendNode"
)
###############

### Backend ###
ament_auto_add_library(backend SHARED
  src/lib/backend.cpp
)

ament_auto_add_library(backend_node SHARED
  src/nodes/backend_node.cpp
)

target_link_libraries(backend_node
  ${Boost_LIBRARIES}
  backend
)

rosidl_target_interfaces(backend_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_nodes(backend_node
  PLUGIN "ichthus_lidar_driver_ros2::backend_node::BackendNode"
)
###############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  cfg
)

