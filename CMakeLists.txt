cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall")

project(Mandoline)

find_package(PkgConfig)
pkg_check_modules(CAIROMM REQUIRED cairomm-1.0)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(Mandoline main.cpp graph.cpp polygon.cpp heuristics.cpp)

target_include_directories(Mandoline PUBLIC "${CAIROMM_INCLUDE_DIRS}")
target_link_libraries(Mandoline PUBLIC "${CAIROMM_LIBRARIES}")
