find_package(Boost REQUIRED COMPONENTS date_time chrono system thread)
find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/protogen/proto)
include_directories(${Boost_INCLUDE_DIRS})