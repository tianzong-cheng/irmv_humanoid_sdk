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

# Listener of the fault-recording acceptance: it subscribes to robot_state on
# one domain and prints every published fault report as JSON. It is a witness
# of the wire, not a recorder, and it links only the protocol and transport
# pieces the runtime publishes through.
add_executable(fault_recording_peer tests/integration/fault_recording_peer.cpp)
target_compile_features(fault_recording_peer PRIVATE cxx_std_17)
target_include_directories(
  fault_recording_peer
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/tests"
)
target_link_libraries(
  fault_recording_peer
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
)
if(IRMV_BUILD_APPLICATION)
  list(APPEND IRMV_INTEGRATION_SCENARIOS
    application_normal
    application_ready_hold
    application_repeat_and_reset
    application_stop_preempt
    application_fault
    application_unknown
    application_overspeed
    application_external_absent
  )
endif()
set(_irmv_scenario_application_args)
if(TARGET application)
  set(_irmv_scenario_application_args
      "--application" "$<TARGET_FILE:application>")
endif()
foreach(scenario IN LISTS IRMV_INTEGRATION_SCENARIOS)
  add_test(
    NAME integration_${scenario}
    COMMAND
      "${Python3_EXECUTABLE}"
      "${PROJECT_SOURCE_DIR}/tools/run_integration.py"
      --scenario "${PROJECT_SOURCE_DIR}/tests/integration/scenarios/${scenario}.json"
      --runtime "$<TARGET_FILE:robot_runtime>"
      --client "$<TARGET_FILE:runtime_client>"
      ${_irmv_scenario_application_args}
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

if(IRMV_BUILD_APPLICATION)
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

endif()
