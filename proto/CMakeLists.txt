find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER message.proto vFirmware_API.proto)

add_library(proto ${PROTO_HEADER} ${PROTO_SRC})
target_link_libraries(proto INTERFACE ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})