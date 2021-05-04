cmake_minimum_required(VERSION 3.1.3)

project(maze_navigating_robot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_transport
        cv_bridge
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf2_ros
        std_srvs
        actionlib
        move_base_msgs
        )

catkin_package()

add_executable(main src/main.cpp src/brick_search.cpp)

target_include_directories(main SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(main PRIVATE include)

target_link_libraries(main ${OpenCV_LIBS} ${catkin_LIBRARIES})

