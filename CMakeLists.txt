cmake_minimum_required(VERSION 3.22)

project(irmv_humanoid_sdk LANGUAGES CXX)

include(CTest)
include(cmake/Dependencies.cmake)

irmv_make_nlohmann_json_available()

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

add_library(
  irmv_configuration
  src/configuration/config.cpp
  src/configuration/sha256.cpp
)
target_compile_features(irmv_configuration PUBLIC cxx_std_17)
target_include_directories(
  irmv_configuration
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(irmv_configuration PRIVATE nlohmann_json::nlohmann_json)

add_executable(config_test tests/unit/config_test.cpp)
target_compile_features(config_test PRIVATE cxx_std_17)
target_include_directories(
  config_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(config_test PRIVATE irmv_configuration)

add_test(NAME config_test COMMAND config_test)
set_tests_properties(config_test PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(
  NAME config_schema_contract_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/unit/config_schema_contract_test.py"
    "${PROJECT_SOURCE_DIR}/config/sdk.schema.json"
    "${PROJECT_SOURCE_DIR}/config/sdk.json"
)
