cmake_minimum_required(VERSION 3.22)

project(irmv_humanoid_sdk LANGUAGES CXX)

include(CTest)
include(cmake/Dependencies.cmake)

add_library(irmv_protocol src/protocol/validation.cpp)
target_compile_features(irmv_protocol PUBLIC cxx_std_17)
target_include_directories(
  irmv_protocol
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

add_executable(protocol_test tests/unit/protocol_test.cpp)
target_compile_features(protocol_test PRIVATE cxx_std_17)
target_include_directories(
  protocol_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(protocol_test PRIVATE irmv_protocol)

add_test(NAME protocol_test COMMAND protocol_test)
