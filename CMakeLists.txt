cmake_minimum_required(VERSION 3.10)

# 设置项目名称和版本
project(CBS VERSION 1.0)

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)

# 打印 OpenCV 版本信息（可选）
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# 包含头文件目录
include_directories(include ${OpenCV_INCLUDE_DIRS})

# 指定源文件
set(SOURCES src/main.cpp)

# 添加源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 添加可执行文件
add_executable(CBS ${SOURCES})

# 链接 OpenCV 库
target_link_libraries(CBS ${OpenCV_LIBS})