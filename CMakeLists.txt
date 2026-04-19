cmake_minimum_required(VERSION 2.8)

project(WORKFORSIN)

# 找 OpenCV
find_package(OpenCV REQUIRED)

# 自动收集 src 目录下所有 cpp
file(GLOB SOURCES "src/*.cpp")

# 生成可执行文件
add_executable(main ${SOURCES})

# 链接 OpenCV
target_link_libraries(main PRIVATE ${OpenCV_LIBS})

# 头文件路径
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})