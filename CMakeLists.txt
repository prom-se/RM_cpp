cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(RM_cpp)

# 寻找库并添加头文件路径
find_package(OpenCV 4.8.0 REQUIRED)
find_package(CSerialPort)
include_directories(${CSerialPort_INCLUDE_DIR})
include_directories(${OpenCV_DIRCTORIES})
include_directories("/opt/MVS/include/")
include_directories("/usr/include/eigen3/")
add_executable(autoaim autoaim.cpp
                autoaim.hpp
                hik_camera/HikCam.cpp hik_camera/HikCam.hpp
                Detector/Detector.cpp Detector/Detector.hpp
                Tracker/Tracker.cpp Tracker/Tracker.hpp
                Tracker/EKF.cpp Tracker/EKF.hpp Tracker/avgFilter.hpp
                visionSerial/src/serial.cpp)

target_link_libraries(autoaim ${OpenCV_LIBS} ${CERES_LIBRARIES} ${CSerialPort_LIBRARY} -pthread)
# 包含海康相机动态链接库
find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
target_link_libraries(autoaim ${MVS_LIB})
