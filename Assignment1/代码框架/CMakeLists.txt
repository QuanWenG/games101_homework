cmake_minimum_required(VERSION 3.10)
project(Rasterizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_executable(
    Rasterizer
    main.cpp
    rasterizer.cpp
    rasterizer.hpp
    Triangle.cpp
    Triangle.hpp
)

target_include_directories(
    Rasterizer PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    Rasterizer PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
)