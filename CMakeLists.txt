cmake_minimum_required(VERSION 2.8)

project(WORKFORSIN)

# 找 OpenCV
find_package(OpenCV REQUIRED)

# 自动收集 src 目录下所有 cpp
file(GLOB SOURCES "src/*.cpp")

# 生成可执行文件
add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})

add_executable(detection src/detection.cpp)
target_link_libraries(detection ${OpenCV_LIBS})

add_executable(pnp src/pnp.cpp)
target_link_libraries(pnp ${OpenCV_LIBS})

