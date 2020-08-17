cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_FLAGS "-g -Wall -Werror -std=c++14")

project(TritonBot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_testing()

find_package(Boost REQUIRED COMPONENTS date_time chrono system thread)
find_package(Armadillo REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER ./proto/vFirmware_API.proto)

add_library(proto ${PROTO_HEADER} ${PROTO_SRC})
target_link_libraries(proto INTERFACE ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(TritonBot ./src/Client.cpp)
target_link_libraries(TritonBot proto ${Armadillo_Libraries} ${Boost_Libraries})

add_executable(TritonBot.exe ./src/Client.cpp)
target_link_libraries(TritonBot.exe proto ${Armadillo_Libraries} ${Boost_Libraries})

add_subdirectory(googletest)
add_subdirectory(test)





