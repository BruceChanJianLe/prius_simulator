cmake_minimum_required(VERSION 3.8)
project(prius_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(prius_description REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

install(DIRECTORY launch models worlds
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
