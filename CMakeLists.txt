cmake_minimum_required(VERSION 3.15)

project(Mandoline)

find_package(PkgConfig)
pkg_check_modules(CAIROMM REQUIRED cairomm-1.0)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(Mandoline main.cpp graph.cpp heuristics.cpp)

set_target_properties(Mandoline PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

target_include_directories(Mandoline PUBLIC "${CAIROMM_INCLUDE_DIRS}")
target_link_libraries(Mandoline PUBLIC "${CAIROMM_LIBRARIES}" Eigen3::Eigen)
