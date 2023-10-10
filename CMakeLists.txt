cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(RM_cpp)

# 寻找库并添加头文件路径
find_package(OpenCV REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${InferenceEngine_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories("/usr/include/eigen3/")
include_directories("/opt/MVS/include/")
link_directories(/usr/lib/x86_64-linux-gnu)

add_executable(autoaim autoaim.cpp
                autoaim.hpp
        hik_camera/HikCam.cpp
        hik_camera/HikCam.hpp
                Detector/Detector.cpp Detector/Detector.hpp
                Tracker/Tracker.cpp Tracker/Tracker.hpp
                Serial/Serial.cpp Serial/Serial.hpp Rune/Rune.cpp Rune/Rune.hpp
        Tracker/EKF.cpp
        Tracker/EKF.hpp)
target_link_libraries(autoaim ${OpenCV_LIBS} ${InferenceEngine_LIBRARIES})
target_link_libraries(autoaim ${CERES_LIBRARIES} -lserialport)
# 包含海康相机动态链接库
find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
target_link_libraries(autoaim ${MVS_LIB})
