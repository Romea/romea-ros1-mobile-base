cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_msgs)

set(${PROJECT_NAME}_CATKIN_DEPS
  std_msgs
  message_generation
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_message_files(
  DIRECTORY msg
  FILES
    KinematicMeasure.msg
    KinematicMeasureStamped.msg
    OneAxleSteeringCommand.msg
    OneAxleSteeringMeasure.msg
    OneAxleSteeringMeasureStamped.msg
    SkidSteeringCommand.msg
    SkidSteeringMeasure.msg
    SkidSteeringMeasureStamped.msg
    OmniSteeringMeasure.msg
    OmniSteeringMeasureStamped.msg
    OmniSteeringCommand.msg
    TwoAxleSteeringMeasure.msg
    TwoAxleSteeringMeasureStamped.msg
    TwoAxleSteeringCommand.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)
