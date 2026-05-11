cmake_minimum_required(VERSION 3.20)
project(getv VERSION 1.0.0 LANGUAGES CXX)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
add_compile_options(-Wall -Wextra -Wpedantic)

# 包含目录
include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED 
    libavcodec 
    libavutil 
    libswscale
    libavformat
)

add_executable(getv
    src/main.cpp
)

target_include_directories(getv PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_libraries(getv 
    ${FFMPEG_LIBRARIES}
    pthread
)