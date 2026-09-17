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

add_library(irmv_hardware src/hardware/fake_backend.cpp)
target_compile_features(irmv_hardware PUBLIC cxx_std_17)
target_include_directories(
  irmv_hardware
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

add_executable(fake_backend_test tests/unit/fake_backend_test.cpp)
target_compile_features(fake_backend_test PRIVATE cxx_std_17)
target_include_directories(
  fake_backend_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(fake_backend_test PRIVATE irmv_hardware)

add_test(NAME fake_backend_test COMMAND fake_backend_test)

add_library(
  irmv_runtime
  src/runtime/safety.cpp
  src/runtime/command_gate.cpp
  src/runtime/execution_fsm.cpp
  src/runtime/states/boot.cpp
  src/runtime/states/zero_torque.cpp
  src/runtime/states/damping.cpp
  src/runtime/states/tracking.cpp
  src/runtime/states/fault.cpp
)
target_compile_features(irmv_runtime PUBLIC cxx_std_17)
target_include_directories(
  irmv_runtime
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  irmv_runtime
  PUBLIC irmv_protocol irmv_configuration irmv_hardware
)

add_executable(safety_test tests/unit/safety_test.cpp)
target_compile_features(safety_test PRIVATE cxx_std_17)
target_include_directories(
  safety_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(safety_test PRIVATE irmv_runtime)

add_test(NAME safety_test COMMAND safety_test)
set_tests_properties(safety_test PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

add_executable(command_gate_test tests/unit/command_gate_test.cpp)
target_compile_features(command_gate_test PRIVATE cxx_std_17)
target_include_directories(
  command_gate_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(command_gate_test PRIVATE irmv_runtime)

add_test(NAME command_gate_test COMMAND command_gate_test)
set_tests_properties(
  command_gate_test PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_executable(execution_fsm_test tests/unit/execution_fsm_test.cpp)
target_compile_features(execution_fsm_test PRIVATE cxx_std_17)
target_include_directories(
  execution_fsm_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(execution_fsm_test PRIVATE irmv_runtime)

add_test(NAME execution_fsm_test COMMAND execution_fsm_test)
set_tests_properties(
  execution_fsm_test PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(
  NAME config_schema_contract_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/unit/config_schema_contract_test.py"
    "${PROJECT_SOURCE_DIR}/config/sdk.schema.json"
    "${PROJECT_SOURCE_DIR}/config/sdk.json"
)
