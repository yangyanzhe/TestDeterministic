cmake_minimum_required(VERSION 3.10)
project(deterministic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

find_package(DART 6.4.0 REQUIRED COMPONENTS collision-bullet utils-urdf gui CONFIG)
include_directories(${DART_INCLUDE_DIRS})
find_package(Boost)
include_directories(${BOOST_INCLUDE_DIR})

set(SOURCE TestDeterministic.h TestDeterministic.cpp main.cpp)
add_executable(deterministic ${SOURCE})
target_link_libraries(deterministic ${DART_LIBRARIES})