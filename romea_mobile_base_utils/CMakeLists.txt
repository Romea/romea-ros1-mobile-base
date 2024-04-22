cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  geometry_msgs
  four_wheel_steering_msgs
  ackermann_msgs

  romea_common_utils
  romea_mobile_base_msgs
  romea_cmd_mux_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
# find_package(romea_cmd_mux_utils REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/control/command_interface.cpp
  # src/conversions/command_conversions.cpp
  # src/conversions/kinematic_conversions.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${romea_core_common_INCLUDE_DIRS}
  ${romea_core_mobile_base_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  romea_core_common::romea_core_common
  romea_core_mobile_base::romea_core_mobile_base
)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#     find_package(ament_lint_auto REQUIRED)
#     list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
#     ament_lint_auto_find_test_dependencies()
#     add_subdirectory(test)
#
# endif()
