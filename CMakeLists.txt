cmake_minimum_required(VERSION 3.5)
project(wiring)

set(CMAKE_CXX_STANDARD 20)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.74 REQUIRED)
message(STATUS "Boost version: ${Boost_VERSION}")

file(GLOB_RECURSE sources_cpp CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE sources_h CONFIGURE_DEPENDS "src/*.h")

add_executable(wiring ${sources_cpp} ${sources_h})

if(NOT TARGET Boost::geometry)
  add_library(Boost::geometry IMPORTED INTERFACE)
  set_property(TARGET Boost::geometry PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                                               ${Boost_INCLUDE_DIR})
  set_property(TARGET Boost::geometry PROPERTY INTERFACE_LINK_LIBRARIES
                                               ${Boost_LIBRARIES})
endif()

target_link_libraries(wiring PUBLIC Boost::geometry)
