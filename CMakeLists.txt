cmake_minimum_required(VERSION 3.22)

project(irmv_humanoid_sdk LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(cmake/Dependencies.cmake)

# Every installed artifact finds its neighbours through a relative rpath and
# never through the build or source directory: a library in <prefix>/lib
# resolves the libraries next to it through $ORIGIN, and the runtime in
# <prefix>/bin resolves them through $ORIGIN/../lib. A dependency that is found
# system-wide is left to the system; the ones that are not are installed next to
# the binary by the installation section at the end of this file.
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# The bundle stays out of the link path: $ORIGIN is the only install rpath entry
# any artifact may have.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

# Optional sanitizer configuration of the first-party targets, used by the CI
# sanitizer job and for local debugging:
#
#   cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug \
#         -DIRMV_SANITIZE=address,undefined
#
# The pinned dependencies are configured by a nested CMake of their own and are
# never instrumented. -fno-sanitize-recover=all turns a sanitizer report into a
# test failure instead of a warning, so a sanitizer run cannot pass while the
# report is only printed.
set(IRMV_SANITIZE
    ""
    CACHE STRING
    "Comma separated sanitizers of the first-party targets, e.g. address,undefined"
)
if(IRMV_SANITIZE)
  message(STATUS "irmv: sanitizing the first-party targets with ${IRMV_SANITIZE}")
  add_compile_options(
    "-fsanitize=${IRMV_SANITIZE}" "-fno-sanitize-recover=all"
    "-fno-omit-frame-pointer" "-g"
  )
  add_link_options("-fsanitize=${IRMV_SANITIZE}")
endif()

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
set_tests_properties(protocol_test PROPERTIES LABELS core)

add_library(
  irmv_configuration
  src/configuration/application_config.cpp
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
set_tests_properties(
  config_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_executable(application_config_test tests/unit/application_config_test.cpp)
target_compile_features(application_config_test PRIVATE cxx_std_17)
target_include_directories(
  application_config_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(application_config_test PRIVATE irmv_configuration)

add_test(NAME application_config_test COMMAND application_config_test)
set_tests_properties(
  application_config_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The policy layer owns the deployment contract a policy ONNX carries and, from
# the next step on, the ONNX Runtime loader and the observation/action
# transformation. It is the only internal library besides the application that
# may link ONNX Runtime, which is what keeps `ldd build/robot_runtime` free of
# it: the lower process never parses a model.
irmv_make_onnxruntime_available()
add_library(
  irmv_policy
  src/policy/action.cpp
  src/policy/deployment.cpp
  src/policy/inference.cpp
  src/policy/observation.cpp
  src/policy/onnx_model.cpp
)
target_compile_features(irmv_policy PUBLIC cxx_std_17)
target_include_directories(
  irmv_policy
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
# ONNX Runtime is linked privately: only this library and the targets that link
# it carry the dependency, and robot_runtime never links irmv_policy.
target_link_libraries(
  irmv_policy
  PUBLIC irmv_protocol irmv_configuration
  PRIVATE nlohmann_json::nlohmann_json IRMV::onnxruntime Threads::Threads
)

add_executable(deployment_test tests/unit/deployment_test.cpp)
target_compile_features(deployment_test PRIVATE cxx_std_17)
target_include_directories(
  deployment_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  deployment_test PRIVATE irmv_policy nlohmann_json::nlohmann_json
)

add_test(NAME deployment_test COMMAND deployment_test)
# The fixtures of the contract live in tests/support/deployment/ and are read
# relative to the source directory, like the configuration tests read
# config/sdk.json.
set_tests_properties(
  deployment_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_library(
  irmv_application
  src/application/behavior_fsm.cpp
  src/application/initialization.cpp
  src/application/input_router.cpp
  src/application/policy_session_record.cpp
  src/application/producer_supervisor.cpp
  src/application/robot_client.cpp
)
target_compile_features(irmv_application PUBLIC cxx_std_17)
target_include_directories(
  irmv_application
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
# The application side of the frozen protocol is deliberately DDS-free: it
# talks to a transport through ClientSink. It links the protocol and the
# configuration, plus the thread runtime and JSON writer of the policy session
# sidecar. It never links irmv_policy: the session record copies the policy
# layer's header-only gain type and calls no policy symbol, so ONNX Runtime
# stays out of this library and out of the lower process.
target_link_libraries(
  irmv_application
  PUBLIC irmv_protocol irmv_configuration
  PRIVATE Threads::Threads nlohmann_json::nlohmann_json
)

add_executable(robot_client_test tests/unit/robot_client_test.cpp)
target_compile_features(robot_client_test PRIVATE cxx_std_17)
target_include_directories(
  robot_client_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(robot_client_test PRIVATE irmv_application irmv_configuration)

add_test(NAME robot_client_test COMMAND robot_client_test)
set_tests_properties(
  robot_client_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_executable(
  application_initialization_test tests/unit/application_initialization_test.cpp
)
target_compile_features(application_initialization_test PRIVATE cxx_std_17)
target_include_directories(
  application_initialization_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
# The test also drives a real ExecutionFsm through the runtime fixture, so it
# links the runtime; the application library itself stays DDS-free.
target_link_libraries(
  application_initialization_test
  PRIVATE irmv_application irmv_configuration irmv_runtime
)

add_test(NAME application_initialization_test COMMAND application_initialization_test)
set_tests_properties(
  application_initialization_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The decision instant of one iteration is a pure function of the host clock
# read after the event drain and the newest receive stamp of those events, so
# it is pinned on its own without DDS, a thread or a wall clock.
add_executable(application_loop_time_test tests/unit/application_loop_time_test.cpp)
target_compile_features(application_loop_time_test PRIVATE cxx_std_17)
target_include_directories(
  application_loop_time_test
  PRIVATE "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(application_loop_time_test PRIVATE irmv_application irmv_protocol)

add_test(NAME application_loop_time_test COMMAND application_loop_time_test)
set_tests_properties(application_loop_time_test PROPERTIES LABELS core)

add_executable(application_fsm_test tests/unit/application_fsm_test.cpp)
target_compile_features(application_fsm_test PRIVATE cxx_std_17)
target_include_directories(
  application_fsm_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
# The behaviour FSM is driven through the official client over a recording sink:
# no DDS, no clock and no thread takes part.
target_link_libraries(
  application_fsm_test PRIVATE irmv_application irmv_configuration
)

add_executable(producer_supervisor_test tests/unit/producer_supervisor_test.cpp)
target_compile_features(producer_supervisor_test PRIVATE cxx_std_17)
target_include_directories(
  producer_supervisor_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(producer_supervisor_test PRIVATE irmv_application)

add_test(NAME producer_supervisor_test COMMAND producer_supervisor_test)
set_tests_properties(
  producer_supervisor_test
  PROPERTIES
    LABELS core
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The application-side policy session record. It links the application library
# for the recorder and the JSON library to parse the sidecar back; no DDS, no
# real clock and no model take part.
add_executable(
  policy_session_record_test tests/unit/policy_session_record_test.cpp
)
target_compile_features(policy_session_record_test PRIVATE cxx_std_17)
target_include_directories(
  policy_session_record_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  policy_session_record_test
  PRIVATE irmv_application Threads::Threads nlohmann_json::nlohmann_json
)

add_test(NAME policy_session_record_test COMMAND policy_session_record_test)
set_tests_properties(
  policy_session_record_test
  PROPERTIES
    LABELS core
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_test(NAME application_fsm_test COMMAND application_fsm_test)
set_tests_properties(
  application_fsm_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_executable(input_router_test tests/unit/input_router_test.cpp)
target_compile_features(input_router_test PRIVATE cxx_std_17)
target_include_directories(
  input_router_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
# The router is the keyboard half of the loop: it links the application library
# for the event type and the configuration for the shipped bindings, and it
# performs no terminal I/O of its own.
target_link_libraries(input_router_test PRIVATE irmv_application irmv_configuration)

add_test(NAME input_router_test COMMAND input_router_test)
set_tests_properties(
  input_router_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_library(irmv_hardware src/hardware/fake_backend.cpp
                          src/hardware/device_lease.cpp)
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
set_tests_properties(fake_backend_test PROPERTIES LABELS core)

add_executable(device_lease_test tests/unit/device_lease_test.cpp)
target_compile_features(device_lease_test PRIVATE cxx_std_17)
target_include_directories(
  device_lease_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(device_lease_test PRIVATE irmv_hardware)

add_test(NAME device_lease_test COMMAND device_lease_test)
set_tests_properties(device_lease_test PROPERTIES LABELS core)

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
set_tests_properties(
  safety_test PROPERTIES LABELS core WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

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
  command_gate_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
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
  execution_fsm_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
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
set_tests_properties(config_schema_contract_test PROPERTIES LABELS core)

# The deployment contract has two independent witnesses: the C++ parser (through
# deployment_test) and the frozen JSON Schema. This check validates the very
# fixtures the parser accepts against the schema, so neither document can drift
# away from the other.
add_test(
  NAME deployment_schema_contract_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/unit/deployment_schema_contract_test.py"
    "${PROJECT_SOURCE_DIR}/schema/onnx-deployment-v1.schema.json"
    "${PROJECT_SOURCE_DIR}/tests/support/deployment"
)
set_tests_properties(deployment_schema_contract_test PROPERTIES LABELS core)

# The policy math is pure: it reads a frozen contract, one snapshot and one
# model action. The fixtures it is exercised against are the very documents the
# contract test parses.
add_executable(policy_math_test tests/unit/policy_math_test.cpp)
target_compile_features(policy_math_test PRIVATE cxx_std_17)
target_include_directories(
  policy_math_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(policy_math_test PRIVATE irmv_policy irmv_configuration)

add_test(NAME policy_math_test COMMAND policy_math_test)
set_tests_properties(
  policy_math_test PROPERTIES LABELS core
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The model fixtures are generated, not checked in: one command builds every
# model from the frozen deployment document, so a binary blob cannot drift away
# from the contract it claims to carry.
set(IRMV_POLICY_FIXTURE_DIR "${CMAKE_CURRENT_BINARY_DIR}/policy-fixtures")
add_custom_command(
  OUTPUT "${IRMV_POLICY_FIXTURE_DIR}/.stamp"
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tools/make_test_model.py"
    "${IRMV_POLICY_FIXTURE_DIR}"
    "${PROJECT_SOURCE_DIR}/tests/support/deployment/valid_three_orders.json"
  COMMAND "${CMAKE_COMMAND}" -E touch "${IRMV_POLICY_FIXTURE_DIR}/.stamp"
  DEPENDS
    "${PROJECT_SOURCE_DIR}/tools/make_test_model.py"
    "${PROJECT_SOURCE_DIR}/tests/support/deployment/valid_three_orders.json"
  COMMENT "Generating the policy model fixtures"
  VERBATIM
)
add_custom_target(policy_model_fixtures ALL
                  DEPENDS "${IRMV_POLICY_FIXTURE_DIR}/.stamp")

add_executable(inference_test tests/unit/inference_test.cpp)
target_compile_features(inference_test PRIVATE cxx_std_17)
target_include_directories(
  inference_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(inference_test PRIVATE irmv_policy Threads::Threads)
add_dependencies(inference_test policy_model_fixtures)

add_test(NAME inference_test COMMAND inference_test "${IRMV_POLICY_FIXTURE_DIR}")
set_tests_properties(inference_test PROPERTIES LABELS core TIMEOUT 120)

add_executable(onnx_model_test tests/unit/onnx_model_test.cpp)
target_compile_features(onnx_model_test PRIVATE cxx_std_17)
target_include_directories(
  onnx_model_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(onnx_model_test PRIVATE irmv_policy)
add_dependencies(onnx_model_test policy_model_fixtures)

add_test(NAME onnx_model_test COMMAND onnx_model_test "${IRMV_POLICY_FIXTURE_DIR}")
set_tests_properties(onnx_model_test PROPERTIES LABELS core)

# The application config test also covers the policy section of the shipped
# configuration, and the config schema contract test the schema that describes
# it.
add_test(
  NAME policy_package_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/policy_package_test.py"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  policy_package_test
  PROPERTIES
    LABELS integration
    TIMEOUT 300
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The acceptance of the local policy: the real application and the real runtime,
# over real DDS, driven through the application's terminal, from a generated
# twelve-joint Fake deployment whose model commands exactly those joints. It is
# the only entry that proves the whole chain - the contract, the loader, the
# worker, the first frame, the authorization and the published target.
# The recorded run, exported: a real runtime records for a few seconds, the
# reader walks the run and the NPZ conversion turns it into an archive the
# consumer can interpret without the chunk format.
add_test(
  NAME record_export_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/record_export_test.py"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  record_export_test
  PROPERTIES
    LABELS integration
    TIMEOUT 300
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

add_test(
  NAME policy_locomotion_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/policy_locomotion_test.py"
    "$<TARGET_FILE:application>"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  policy_locomotion_test
  PROPERTIES
    LABELS integration
    TIMEOUT 300
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The example external producer: a target generator the application selects,
# prepares and authorizes, and that then publishes on the execution command
# topic. It is an example and a test peer, and it links only the frozen protocol
# pieces - the transport's producer and execution codecs, the configuration
# loader and the protocol types - never irmv_application, irmv_telemetry or
# irmv_policy, so nothing of the supervising or ONNX side reaches it and it is
# installed next to the two processes as part of operating the deployment. Its
# link libraries are defined later in this file, which is a plain forward
# reference CMake resolves when it generates.
add_executable(external_producer examples/external_producer.cpp)
target_compile_features(external_producer PRIVATE cxx_std_17)
target_include_directories(
  external_producer
  PRIVATE "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(
  external_producer
  PRIVATE
    irmv_transport
    irmv_configuration
    irmv_protocol
    nlohmann_json::nlohmann_json
    Threads::Threads
)

# The training-side reference sample (§11.1, Task 9): the model the training
# repository exported, and the state/command/observation/action/target it
# recorded for four deterministic cases, compared element by element against
# this SDK's own observation builder, model session and action pipeline. The
# artifact lives in the adjacent training repository (or wherever
# -DIRMV_POLICY_REFERENCE points), because a model and a recorded environment
# are not build inputs of this one: without them the entry is not registered and
# the configure step says so, so their absence is never a silent pass.
set(
  IRMV_POLICY_REFERENCE
  "${PROJECT_SOURCE_DIR}/../irmv_humanoid_locomotion/logs/rsl_rl/irmv_flat/2026-09-08_09-56-44/exported"
  CACHE PATH
  "Directory holding policy.onnx and reference_sample.json from the training repository"
)
if(EXISTS "${IRMV_POLICY_REFERENCE}/policy.onnx"
   AND EXISTS "${IRMV_POLICY_REFERENCE}/reference_sample.json")
  add_test(
    NAME policy_reference_test
    COMMAND
      "$<TARGET_FILE:verify_policy_sample>"
      "${IRMV_POLICY_REFERENCE}/reference_sample.json"
      "${IRMV_POLICY_REFERENCE}/policy.onnx"
  )
  set_tests_properties(
    policy_reference_test
    PROPERTIES
      LABELS integration
      TIMEOUT 120
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  )
  message(
    STATUS
      "irmv: the training-side reference sample of ${IRMV_POLICY_REFERENCE} is registered as policy_reference_test"
  )
else()
  message(
    STATUS
      "irmv: no training-side reference sample in ${IRMV_POLICY_REFERENCE}; "
      "policy_reference_test is not registered (see docs/testing.md)"
  )
endif()

# The external producer acceptance: the real application, the real runtime and
# the real example producer process. It is the only entry that proves the whole
# external lifecycle - registration, preparation, the confirmation that
# authorizes the producer, the producer's own frame stream under the identity
# the runtime granted, and the stop that ends it - and it is a driver rather
# than a scenario file because the scenario harness drives `runtime_client`,
# which speaks the supervisor protocol and not a producer lifecycle.
if(NOT TARGET external_producer)
  message(
    FATAL_ERROR
      "the external producer acceptance needs the `external_producer` target "
      "(examples/external_producer.cpp)"
  )
endif()
add_test(
  NAME external_producer_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/external_producer_test.py"
    "$<TARGET_FILE:application>"
    "$<TARGET_FILE:robot_runtime>"
    "$<TARGET_FILE:external_producer>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  external_producer_test
  PROPERTIES
    LABELS integration
    TIMEOUT 300
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

find_package(Threads REQUIRED)

add_library(
  irmv_telemetry
  src/telemetry/diagnostic_write.cpp
  src/telemetry/logger.cpp
  src/telemetry/record_reader.cpp
  src/telemetry/recorder.cpp
)
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
  PRIVATE Threads::Threads nlohmann_json::nlohmann_json
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

add_executable(recorder_test tests/unit/recorder_test.cpp)
target_compile_features(recorder_test PRIVATE cxx_std_17)
target_include_directories(
  recorder_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  recorder_test PRIVATE irmv_telemetry Threads::Threads
                       nlohmann_json::nlohmann_json
)

add_test(NAME recorder_test COMMAND recorder_test)
set_tests_properties(recorder_test PROPERTIES LABELS core TIMEOUT 120)

add_executable(record_reader_test tests/unit/record_reader_test.cpp)
target_compile_features(record_reader_test PRIVATE cxx_std_17)
target_include_directories(
  record_reader_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  record_reader_test PRIVATE irmv_telemetry Threads::Threads
                            nlohmann_json::nlohmann_json
)

add_test(NAME record_reader_test COMMAND record_reader_test)
set_tests_properties(record_reader_test PROPERTIES LABELS core TIMEOUT 120)
set_tests_properties(recorder_test PROPERTIES LABELS core TIMEOUT 120)

add_test(NAME logger_test COMMAND logger_test)
set_tests_properties(
  logger_test PROPERTIES LABELS core WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
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
# The producer interface shares the execution module's types (the identity, the
# bounded texts, JointCommand and StopReason), so it includes execution.idl
# rather than restating them: one definition of the frame both protocols carry.
idlcxx_generate(
  TARGET producer_idl
  FILES "${PROJECT_SOURCE_DIR}/protocol/producer.idl"
  INCLUDES "${PROJECT_SOURCE_DIR}/protocol"
  WARNINGS no-implicit-extensibility
)

add_library(
  irmv_transport
  src/transport/client_transport.cpp
  src/transport/codec.cpp
  src/transport/dds_transport.cpp
  src/transport/producer_codec.cpp
  src/transport/producer_transport.cpp
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
    producer_idl
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
# The bounded-channel handoff is deterministic: it never needs a peer, so it is
# part of the core suite and runs under the sanitizers as well.
set_tests_properties(channel_test PROPERTIES LABELS core)

add_executable(producer_codec_test tests/integration/producer_codec_test.cpp)
target_compile_features(producer_codec_test PRIVATE cxx_std_17)
target_include_directories(
  producer_codec_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(producer_codec_test PRIVATE irmv_transport)

add_test(NAME producer_codec_test COMMAND producer_codec_test)
set_tests_properties(
  producer_codec_test
  PROPERTIES
    LABELS integration
    TIMEOUT 60
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

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

# The read tool of a recorded run: it walks one run directory and prints what it
# holds. It only reads - no DDS, no device - and it is installed with the two
# processes because reading a run back is part of operating one.
# The SDK half of the training-side reference sample: one state and one velocity
# command, pushed through this deployment's own observation builder, model and
# action pipeline, compared element by element with the numbers the training
# environment recorded (docs/testing.md). It needs a model and a reference
# document, so it is a tool rather than a test of its own, and the test entry
# below registers it when those artifacts are present.
add_executable(verify_policy_sample tools/verify_policy_sample.cpp)
target_compile_features(verify_policy_sample PRIVATE cxx_std_17)
target_include_directories(
  verify_policy_sample PRIVATE "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(
  verify_policy_sample PRIVATE irmv_policy nlohmann_json::nlohmann_json
)
install(
  TARGETS verify_policy_sample RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

add_executable(read_record tools/read_record.cpp)
target_compile_features(read_record PRIVATE cxx_std_17)
target_include_directories(
  read_record PRIVATE "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(
  read_record PRIVATE irmv_telemetry nlohmann_json::nlohmann_json
)

# The application: the supervising upper process of a separate robot_runtime.
# It owns the supervision loop no other target needs, so its loop source is
# compiled into the executable instead of a library of its own. It links the
# application host libraries and never irmv_runtime (the lower layer is another
# process, reached through DDS alone) and never irmv_telemetry (phase two ships
# no application logging; the status goes to stdout).
add_executable(application src/application/main.cpp
                           src/application/application_loop.cpp)
target_compile_features(application PRIVATE cxx_std_17)
target_include_directories(
  application
  PRIVATE "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(
  application
  PRIVATE
    irmv_application
    irmv_transport
    irmv_configuration
    irmv_protocol
    # The policy layer is linked into the application executable and into
    # nothing else: ONNX Runtime reaches the supervising process and never the
    # runtime process, which the build verifies afterwards with `ldd`.
    irmv_policy
)

add_test(
  NAME application_cli_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/application_cli_test.py"
    "$<TARGET_FILE:application>"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  application_cli_test
  PROPERTIES
    LABELS integration
    TIMEOUT 180
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The official client against a real standalone runtime: the same transport the
# application ships, driven by hand so the arrival order of an answer and its
# witness state is deterministic.
add_executable(application_client_test tests/integration/application_client_test.cpp)
target_compile_features(application_client_test PRIVATE cxx_std_17)
target_include_directories(
  application_client_test
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  application_client_test
  PRIVATE
    irmv_application
    irmv_transport
    irmv_configuration
    irmv_protocol
    nlohmann_json::nlohmann_json
    Threads::Threads
)

add_test(
  NAME application_client_test
  COMMAND
    application_client_test "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  application_client_test
  PROPERTIES
    LABELS integration
    TIMEOUT 300
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
# `--application` is the real upper process: the phase-1 scenarios ignore it,
# the application_* scenarios drive it through a PTY, and it is never simulated
# by the test client.
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
  stderr_backpressure
  local_stop
  write_refused
  application_normal
  application_ready_hold
  application_repeat_and_reset
  application_stop_preempt
  application_fault
  application_unknown
  application_overspeed
  application_external_absent
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
      --application "$<TARGET_FILE:application>"
      --source "${PROJECT_SOURCE_DIR}"
      # The runner isolates each scenario under <work-dir>/<scenario> and
      # claims its domain lock one level up, in the shared parent, which is
      # where every scenario looks for it (choose_domain's default).
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

# The cross-process phase acceptance: the cases one scenario run cannot express
# (a runtime replaced under a running application, a SIGSTOPped runtime, a
# paused and a killed application, the Locomotion boundary and the survivor
# check). It reuses the harness for the cases a scenario file describes -
# application_normal is invoked as the subprocess it is - and drives the rest
# itself over real DDS, always with the real `application` binary.
add_test(
  NAME application_flow_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/application_flow_test.py"
    "$<TARGET_FILE:application>"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  application_flow_test
  PROPERTIES
    LABELS integration
    TIMEOUT 600
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The state-switching stability run: the real application and the real runtime
# cycling through the documented behaviours (Damping, Initialize to Ready,
# ZeroTorque back to Standby), with the memory, thread and descriptor counts of
# both processes sampled from /proc. This entry is the short regression guard;
# the thirty minutes of the phase-3 acceptance are the same driver with
# `--duration-s 1800`, and docs/testing.md records the run and its numbers.
add_test(
  NAME application_stability_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/application_stability_test.py"
    "$<TARGET_FILE:application>"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
    --duration-s 20
    --sample-s 5
    --domain 221
    --work-dir "${CMAKE_CURRENT_BINARY_DIR}/stability"
)
set_tests_properties(
  application_stability_test
  PROPERTIES
    LABELS integration
    TIMEOUT 180
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# The application-driven recorded run (§11.1, Task 9): the real application
# supervises a session that the real runtime records with the recorder enabled,
# and the run is read back as the evidence of what the machine did - the command
# in force in every recorded tick, the feedback and the output of that same tick,
# and the authorization context in the run's own session table.
add_test(
  NAME application_record_test
  COMMAND
    "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/integration/application_record_test.py"
    "$<TARGET_FILE:application>"
    "$<TARGET_FILE:robot_runtime>"
    "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  application_record_test
  PROPERTIES
    LABELS integration
    TIMEOUT 300
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------

# `format` rewrites every first-party C/C++ file of include/, src/ and tests/ in
# place; `format-check` is read-only and fails on the first diff, which is what
# CI runs. Both go through tools/format.py, which owns the file list and refuses
# to run with any clang-format other than the pinned 18.1.8: it reports the
# version and the path it found and points at tools/requirements-format.txt. A
# missing or wrong-versioned tool therefore fails these targets with that
# message instead of producing an empty pass that proves nothing.
find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_custom_target(
  format
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/format.py" --write
  COMMENT "Formatting the first-party C/C++ tree with clang-format 18.1.8"
  USES_TERMINAL
  VERBATIM
)

add_custom_target(
  format-check
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/format.py" --check
  COMMENT "Checking the first-party C/C++ tree with clang-format 18.1.8"
  USES_TERMINAL
  VERBATIM
)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

# Installed layout of the two processes:
#
#   <prefix>/bin/robot_runtime      the lower layer
#   <prefix>/bin/application        the supervising upper layer
#   <prefix>/lib/                   the shared libraries they need and that are
#                                   not provided system-wide
#   <prefix>/etc/irmv/sdk.json      the shipped default Fake configuration
#   <prefix>/etc/irmv/sdk.schema.json
#
# The test clients of this phase are never installed.
install(TARGETS robot_runtime RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS read_record RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS application RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS external_producer RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

# The configuration of the default Fake deployment and the schema that describes
# it: the schema is part of the configuration contract, not of a test.
install(
  FILES "${PROJECT_SOURCE_DIR}/config/sdk.json"
        "${PROJECT_SOURCE_DIR}/config/sdk.schema.json"
  DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/irmv"
)

# <prefix>/bin/robot_runtime resolves <prefix>/lib through this relative entry.
# Every other target keeps the plain `$ORIGIN` set at the top of this file, so no
# installed artifact carries a build or source path in its rpath.
set_target_properties(
  robot_runtime
  PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
)
# The application resolves its libraries the same way: it is built from the same
# tree, links the same pinned DDS libraries and is started from the same bin.
set_target_properties(
  application
  PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
)
# The example producer links the same pinned DDS libraries the application does
# (its transport), so the installed binary resolves them from <prefix>/lib
# exactly like the two processes and runs from a tree that was moved away from
# its build directory. The reference-sample tool links ONNX Runtime for the same
# reason the application does, and needs the same entry.
set_target_properties(
  external_producer
  PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
)
set_target_properties(
  verify_policy_sample
  PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
)

# The installed tree has to run after being moved or copied to another machine
# and must never reference the build or source directory. Every shared library
# the runtime resolves outside the platform directories - here the pinned
# CycloneDDS C++ binding and its C runtime, which live in a private prefix - is
# installed into <prefix>/lib together with its soname symlinks. A dependency
# found system-wide is left to the system instead of being duplicated. The
# pinned dependency libraries carry no absolute rpath of their own (the C++
# binding none, the C runtime a relative $ORIGIN/../lib), and the installation
# verifies below that the installed runtime carries exactly that one relative
# entry.
#
# The scan runs on the build-tree binaries, whose rpath still resolves the
# private prefixes of the dependencies; the installed copies are only written
# afterwards. Every private dependency is named as a search directory, so the
# resolution never depends on the build rpath: the pinned CycloneDDS C++ binding
# carries no rpath of its own (its C runtime is only found through its
# directory) and the pinned ONNX Runtime lives in a prefix of its own as well.
# This is the Linux layout ($ORIGIN, ELF dependencies), which is the only
# platform this phase targets.
install(
  CODE "
set(_irmv_libdir \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\")
file(
  GET_RUNTIME_DEPENDENCIES
  EXECUTABLES \"$<TARGET_FILE:robot_runtime>\"
              \"$<TARGET_FILE:application>\"
  DIRECTORIES \"$<TARGET_FILE_DIR:CycloneDDS-CXX::ddscxx>\"
            \"$<TARGET_FILE_DIR:CycloneDDS::ddsc>\"
            \"${IRMV_ONNXRUNTIME_ROOT}/lib\"
  RESOLVED_DEPENDENCIES_VAR _irmv_resolved
  UNRESOLVED_DEPENDENCIES_VAR _irmv_unresolved
  PRE_EXCLUDE_REGEXES
    \"^libc[.]so\" \"^libm[.]so\" \"^libdl[.]so\" \"^librt[.]so\"
    \"^libpthread[.]so\" \"^libgcc_s[.]so\" \"^libstdc[+][+]\" \"^libatomic[.]so\"
    \"^ld-linux\"
  POST_EXCLUDE_REGEXES
    \"^/lib/\" \"^/lib64/\" \"^/usr/lib/\" \"^/usr/lib64/\" \"^/usr/local/lib/\"
  POST_INCLUDE_REGEXES \"^libonnxruntime\" \"^libddsc\"
)
if(_irmv_unresolved)
  message(
    FATAL_ERROR
      \"the installed runtime would miss shared libraries: \${_irmv_unresolved}\"
  )
endif()
foreach(_irmv_library IN LISTS _irmv_resolved)
  get_filename_component(_irmv_name \"\${_irmv_library}\" NAME)
  file(
    INSTALL
    DESTINATION \"\${_irmv_libdir}\"
    TYPE SHARED_LIBRARY
    FILES \"\${_irmv_library}\"
    FOLLOW_SYMLINK_CHAIN
  )
  message(STATUS \"irmv install: bundled \${_irmv_name}\")
endforeach()

# The installed runtime and the installed application resolve their libraries
# through the relative rpath and through nothing else: an installation that
# leaked a link-path or build-tree directory into that entry fails here instead
# of producing a tree that only works next to the source it was built from.
file(
  RPATH_CHECK
  FILE \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/robot_runtime\"
  RPATH \"\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}\"
)
file(
  RPATH_CHECK
  FILE \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/application\"
  RPATH \"\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}\"
)
file(
  RPATH_CHECK
  FILE \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/external_producer\"
  RPATH \"\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}\"
)
file(
  RPATH_CHECK
  FILE \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/verify_policy_sample\"
  RPATH \"\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}\"
)
"
)
