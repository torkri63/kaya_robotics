cmake_minimum_required(VERSION 3.8)
project(kaya_msgs)

# Enable warning flags for C++ (not strictly needed for messages)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)    # ⬅️ add this

# Generate the CubeInfo message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubeInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs            # ⬅️ add this
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
