cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_FLAGS "-g -Wall -Werror -std=c++14")

project(TritonBot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_testing()

add_subdirectory(googletest)
add_subdirectory(proto)
add_subdirectory(src)
add_subdirectory(test)





