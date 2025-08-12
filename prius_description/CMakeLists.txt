cmake_minimum_required(VERSION 3.8)
project(prius_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY launch models urdf
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
