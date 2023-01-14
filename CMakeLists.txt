cmake_minimum_required(VERSION 3.18)
project(wiring)

set(CMAKE_CXX_STANDARD 20)

file(GLOB_RECURSE sources_cpp CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE sources_h CONFIGURE_DEPENDS "src/*.h")
add_executable(wiring ${sources_cpp} ${sources_h})