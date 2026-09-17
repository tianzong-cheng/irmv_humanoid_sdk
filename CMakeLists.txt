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

find_package(Threads REQUIRED)

add_library(irmv_telemetry src/telemetry/logger.cpp)
target_compile_features(irmv_telemetry PUBLIC cxx_std_17)
target_include_directories(
  irmv_telemetry
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  irmv_telemetry
  PUBLIC irmv_protocol irmv_configuration
  PRIVATE Threads::Threads
)

add_executable(logger_test tests/unit/logger_test.cpp)
target_compile_features(logger_test PRIVATE cxx_std_17)
target_include_directories(
  logger_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(logger_test PRIVATE irmv_telemetry)

add_test(NAME logger_test COMMAND logger_test)
set_tests_properties(
  logger_test PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The DDS dependency is only needed by the transport targets below and is never
# skipped silently: a missing or mismatched CycloneDDS installation is a
# configuration error.
irmv_make_cyclonedds_available()
idlcxx_generate(
  TARGET execution_idl
  FILES "${PROJECT_SOURCE_DIR}/protocol/execution.idl"
  WARNINGS no-implicit-extensibility
)

add_library(
  irmv_transport
  src/transport/codec.cpp
  src/transport/dds_transport.cpp
)
target_compile_features(irmv_transport PUBLIC cxx_std_17)
target_include_directories(
  irmv_transport
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  irmv_transport
  PUBLIC
    irmv_protocol
    irmv_configuration
    CycloneDDS-CXX::ddscxx
    execution_idl
  PRIVATE Threads::Threads
)

add_executable(channel_test tests/unit/channel_test.cpp)
target_compile_features(channel_test PRIVATE cxx_std_17)
target_include_directories(
  channel_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(channel_test PRIVATE irmv_transport)

add_test(NAME channel_test COMMAND channel_test)

add_executable(dds_codec_test tests/integration/dds_codec_test.cpp)
target_compile_features(dds_codec_test PRIVATE cxx_std_17)
target_include_directories(
  dds_codec_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(dds_codec_test PRIVATE irmv_transport)

add_test(NAME dds_codec_test COMMAND dds_codec_test)
set_tests_properties(
  dds_codec_test
  PROPERTIES
    LABELS integration
    TIMEOUT 30
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# Standalone fake deployment: the only executable entry point of this phase.
# It owns the control loop no other target needs, so its loop source is compiled
# into the executable instead of a library of its own.
add_executable(robot_runtime src/runtime/main.cpp src/runtime/runtime_loop.cpp)
target_compile_features(robot_runtime PRIVATE cxx_std_17)
target_include_directories(
  robot_runtime
  PRIVATE "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(
  robot_runtime
  PRIVATE
    irmv_runtime
    irmv_transport
    irmv_telemetry
    nlohmann_json::nlohmann_json
)

add_test(
  NAME runtime_cli_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/runtime_cli_test.py"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  runtime_cli_test
  PROPERTIES
    LABELS integration
    TIMEOUT 180
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# Confirmation helper of the integration tests: pure protocol logic, no DDS.
add_executable(confirmation_test tests/integration/confirmation_test.cpp)
target_compile_features(confirmation_test PRIVATE cxx_std_17)
target_include_directories(
  confirmation_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(confirmation_test PRIVATE irmv_protocol)

add_test(NAME confirmation_test COMMAND confirmation_test)
set_tests_properties(
  confirmation_test
  PROPERTIES
    LABELS integration
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# Real-DDS test client of the scenario matrix. It speaks the frozen protocol on
# the same topics and the same interface as the runtime; nothing of it is part
# of the shipped runtime.
add_executable(runtime_client tests/integration/runtime_client.cpp)
target_compile_features(runtime_client PRIVATE cxx_std_17)
target_include_directories(
  runtime_client
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  runtime_client
  PRIVATE
    irmv_transport
    irmv_configuration
    irmv_protocol
    nlohmann_json::nlohmann_json
    Threads::Threads
)

# Every scenario file is one independent CTest entry under the `integration`
# label; the harness isolates the DDS domain of each entry below 230 itself.
set(
  IRMV_INTEGRATION_SCENARIOS
  normal
  request_retry
  stale_context
  supervisor_killed
  supervisor_paused
  producer_paused
  runtime_restarted
  command_flood
  slow_logger
)
foreach(scenario IN LISTS IRMV_INTEGRATION_SCENARIOS)
  add_test(
    NAME integration_${scenario}
    COMMAND
      "${Python3_EXECUTABLE}"
      "${PROJECT_SOURCE_DIR}/tools/run_integration.py"
      --scenario "${PROJECT_SOURCE_DIR}/tests/integration/scenarios/${scenario}.json"
      --runtime "$<TARGET_FILE:robot_runtime>"
      --client "$<TARGET_FILE:runtime_client>"
      --source "${PROJECT_SOURCE_DIR}"
      --work-dir "${CMAKE_CURRENT_BINARY_DIR}/integration"
  )
  set_tests_properties(
    integration_${scenario}
    PROPERTIES
      LABELS integration
      TIMEOUT 300
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  )
endforeach()
