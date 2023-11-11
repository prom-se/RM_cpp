cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 11)

project(RM_cpp)

# 寻找库并添加头文件路径
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIRCTORIES})
include_directories("/opt/MVS/include/")
include_directories("/usr/include/eigen3/")
add_executable(autoaim autoaim.cpp
                autoaim.hpp
                hik_camera/HikCam.cpp hik_camera/HikCam.hpp
                Detector/Detector.cpp Detector/Detector.hpp
                Tracker/Tracker.cpp Tracker/Tracker.hpp
                Tracker/EKF.cpp Tracker/EKF.hpp Tracker/avgFilter.hpp
                Serial/Serial.cpp Serial/Serial.hpp)

target_link_libraries(autoaim ${OpenCV_LIBS} ${CERES_LIBRARIES} -lserialport -pthread)
# 包含海康相机动态链接库
find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
target_link_libraries(autoaim ${MVS_LIB})
