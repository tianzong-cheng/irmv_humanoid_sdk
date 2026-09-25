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

add_library(
  irmv_application
  src/application/behavior_fsm.cpp
  src/application/initialization.cpp
  src/application/input_router.cpp
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
# talks to a transport through ClientSink, so this library links the protocol
# and the configuration and nothing else.
target_link_libraries(irmv_application PUBLIC irmv_protocol irmv_configuration)

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
set_tests_properties(fake_backend_test PROPERTIES LABELS core)

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

find_package(Threads REQUIRED)

add_library(
  irmv_telemetry
  src/telemetry/diagnostic_write.cpp
  src/telemetry/logger.cpp
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

add_library(
  irmv_transport
  src/transport/client_transport.cpp
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
# The bounded-channel handoff is deterministic: it never needs a peer, so it is
# part of the core suite and runs under the sanitizers as well.
set_tests_properties(channel_test PROPERTIES LABELS core)

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
install(TARGETS application RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

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
# The scan runs on the build-tree binary, whose rpath still resolves the private
# prefix of the dependency; the installed copy of the runtime is only written
# afterwards. The two DDS libraries are named as search directories as well: the
# pinned C++ binding carries no rpath of its own, so the C runtime it needs is
# only found through its directory. This is the Linux layout ($ORIGIN, ELF
# dependencies), which is the only platform this phase targets.
install(
  CODE "
set(_irmv_libdir \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\")
file(
  GET_RUNTIME_DEPENDENCIES
  EXECUTABLES \"$<TARGET_FILE:robot_runtime>\"
              \"$<TARGET_FILE:application>\"
  DIRECTORIES \"$<TARGET_FILE_DIR:CycloneDDS-CXX::ddscxx>\"
            \"$<TARGET_FILE_DIR:CycloneDDS::ddsc>\"
  RESOLVED_DEPENDENCIES_VAR _irmv_resolved
  UNRESOLVED_DEPENDENCIES_VAR _irmv_unresolved
  PRE_EXCLUDE_REGEXES
    \"^libc[.]so\" \"^libm[.]so\" \"^libdl[.]so\" \"^librt[.]so\"
    \"^libpthread[.]so\" \"^libgcc_s[.]so\" \"^libstdc[+][+]\" \"^libatomic[.]so\"
    \"^ld-linux\"
  POST_EXCLUDE_REGEXES
    \"^/lib/\" \"^/lib64/\" \"^/usr/lib/\" \"^/usr/lib64/\" \"^/usr/local/lib/\"
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
"
)
