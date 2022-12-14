cmake_minimum_required(VERSION 2.8.3)
project(zed_open_capture_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        sensor_msgs
        std_msgs
        )

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

find_package(OpenCV)
find_package(Boost REQUIRED)
find_package(HIDAPI REQUIRED)

catkin_package()

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
        ${HIDAPI_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(zed_node
        src/zed_open_capture_node.cpp
        src/zed_open_capture_wrapper.cpp
        src/videocapture.cpp
        src/sensorcapture.cpp
        )

target_compile_definitions(zed_node PUBLIC VIDEO_MOD_AVAILABLE)
target_compile_definitions(zed_node PUBLIC SENSORS_MOD_AVAILABLE)

target_link_libraries(zed_node
        ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES}  ${HIDAPI_LIBRARIES})
