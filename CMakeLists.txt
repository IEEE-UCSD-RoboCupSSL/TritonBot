cmake_minimum_required(VERSION 3.16)


set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

project(TritonBot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_testing()

AUX_SOURCE_DIRECTORY(./src SRCFILES)

find_package(Boost REQUIRED COMPONENTS date_time chrono system thread log)
find_package(Armadillo REQUIRED)
find_package(Protobuf REQUIRED)

add_definitions(-DBOOST_LOG_DYN_LINK)

include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)
include_directories(${Protobuf_INCLUDE_DIRS})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER ./proto/vFirmware_API.proto)

add_library(proto ${PROTO_HEADER} ${PROTO_SRC})
target_link_libraries(proto INTERFACE ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(TritonBot ./Client.cpp ${SRCFILES})
target_link_libraries(TritonBot PUBLIC proto ${Armadillo_Libraries} ${Boost_Libraries})

add_executable(TritonBot.exe ./Client.cpp ${SRCFILES})
target_link_libraries(TritonBot.exe PUBLIC proto ${Armadillo_Libraries} ${Boost_Libraries})

add_subdirectory(googletest)
add_subdirectory(test)

