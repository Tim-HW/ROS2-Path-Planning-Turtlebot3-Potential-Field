cmake_minimum_required(VERSION 3.5)
project(potential_field)

# Set default C and C++ standards if not specified
set(CMAKE_C_STANDARD 99 CACHE STRING "C Standard to use")
set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ Standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compile options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Define the sources (automatically include all .cpp files in src directory)
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Add executable for the node
add_executable(potential_field_node ${SOURCES})
target_include_directories(potential_field_node PUBLIC include)
ament_target_dependencies(
  potential_field_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
)


# Add install rules for the node
install(
  TARGETS potential_field_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if the directory exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Automatically discover lint tests
  ament_lint_auto_find_test_dependencies()
  
  # Optionally uncomment to skip copyright/license check
  # set(ament_cmake_copyright_FOUND TRUE)
  
  # Optionally uncomment to skip cpplint (useful in non-Git environments)
  # set(ament_cmake_cpplint_FOUND TRUE)
endif()

# Generate and install package.xml for ROS 2
ament_package()
