# Real hardware dependencies.
#
# The default build is fake-only: it neither fetches nor links the vendor CAN-FD
# library, the upstream runtime I/O or the IMU driver. Turning
# IRMV_ENABLE_REAL_BACKEND on makes those dependencies required and fails the
# configuration when their provenance cannot be checked; there is no empty-value
# escape hatch.
#
# The pins are read from deps/hardware-pins.json. The cache variables below may
# only repeat a manifest value: a different value is an error, not an override.

include_guard(GLOBAL)

set(IRMV_IRMV_CONTROL_SOURCE_DIR
    "${PROJECT_SOURCE_DIR}/deps/irmv_control"
    CACHE PATH "IRMVControl checkout that provides IRMVControl::runtime_io")

set(IRMV_YIS_IMU_SOURCE_DIR
    "${PROJECT_SOURCE_DIR}/deps/yis_imu_driver"
    CACHE PATH "Pinned yis_imu_driver checkout")

set(IRMV_IRMV_CONTROL_COMMIT
    ""
    CACHE STRING "Optional repeat of the manifest IRMVControl commit; must match")

set(IRMV_YIS_IMU_COMMIT
    ""
    CACHE STRING "Optional repeat of the manifest YIS commit; must match")

set(IRMV_PINS_FILE
    "${PROJECT_SOURCE_DIR}/deps/hardware-pins.json"
    CACHE FILEPATH "Manifest of the real dependency pins")

function(irmv_read_pins out_irmv_commit out_yis_commit out_vendor_sha
         out_vendor_machine)
  set(pins_file "${IRMV_PINS_FILE}")
  if(NOT EXISTS "${pins_file}")
    message(FATAL_ERROR "${pins_file} is missing; real dependencies cannot be verified")
  endif()
  file(READ "${pins_file}" pins)
  string(JSON irmv_commit GET "${pins}" irmv_control commit)
  string(JSON yis_commit GET "${pins}" yis_imu_driver commit)
  string(JSON vendor_sha GET "${pins}" vendor_library sha256)
  string(JSON vendor_machine GET "${pins}" vendor_library elf_machine)
  set(${out_irmv_commit}
      "${irmv_commit}"
      PARENT_SCOPE)
  set(${out_yis_commit}
      "${yis_commit}"
      PARENT_SCOPE)
  set(${out_vendor_sha}
      "${vendor_sha}"
      PARENT_SCOPE)
  set(${out_vendor_machine}
      "${vendor_machine}"
      PARENT_SCOPE)
endfunction()

function(irmv_require_checkout option directory commit)
  if(NOT IS_DIRECTORY "${directory}")
    message(FATAL_ERROR "${option} does not exist: ${directory}")
  endif()
  execute_process(
    COMMAND git -C "${directory}" rev-parse HEAD
    RESULT_VARIABLE git_result
    OUTPUT_VARIABLE actual_commit
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(NOT git_result EQUAL 0)
    message(FATAL_ERROR "${option} is not a git checkout: ${directory}")
  endif()
  if(NOT actual_commit STREQUAL commit)
    message(
      FATAL_ERROR
        "${option} is at ${actual_commit}, not the pinned commit ${commit}")
  endif()
endfunction()

function(irmv_require_real_dependency out_vendor_path out_vendor_dir)
  irmv_read_pins(irmv_commit yis_commit vendor_sha vendor_machine)

  if(IRMV_IRMV_CONTROL_COMMIT AND NOT IRMV_IRMV_CONTROL_COMMIT STREQUAL
     irmv_commit)
    message(
      FATAL_ERROR
        "IRMV_IRMV_CONTROL_COMMIT=${IRMV_IRMV_CONTROL_COMMIT} does not match the manifest pin ${irmv_commit}"
    )
  endif()
  if(IRMV_YIS_IMU_COMMIT AND NOT IRMV_YIS_IMU_COMMIT STREQUAL yis_commit)
    message(
      FATAL_ERROR
        "IRMV_YIS_IMU_COMMIT=${IRMV_YIS_IMU_COMMIT} does not match the manifest pin ${yis_commit}"
    )
  endif()

  irmv_require_checkout(IRMV_IRMV_CONTROL_SOURCE_DIR
                        "${IRMV_IRMV_CONTROL_SOURCE_DIR}" "${irmv_commit}")
  irmv_require_checkout(IRMV_YIS_IMU_SOURCE_DIR
                        "${IRMV_YIS_IMU_SOURCE_DIR}" "${yis_commit}")

  if(NOT EXISTS
     "${IRMV_IRMV_CONTROL_SOURCE_DIR}/include/hardware/runtime_io.h")
    message(
      FATAL_ERROR
        "${IRMV_IRMV_CONTROL_SOURCE_DIR} does not provide include/hardware/runtime_io.h"
    )
  endif()

  set(vendor
      "${IRMV_IRMV_CONTROL_SOURCE_DIR}/ThirdParty/controlcanfd/libs/libcontrolcanfd.so"
  )
  if(NOT EXISTS "${vendor}")
    message(FATAL_ERROR "vendor library is missing: ${vendor}")
  endif()
  file(SHA256 "${vendor}" actual_sha)
  if(NOT actual_sha STREQUAL vendor_sha)
    message(
      FATAL_ERROR
        "vendor library SHA-256 ${actual_sha} does not match the manifest ${vendor_sha}"
    )
  endif()
  execute_process(
    COMMAND "${Python3_EXECUTABLE}"
            "${PROJECT_SOURCE_DIR}/tools/check_elf_machine.py" "${vendor}"
            "${vendor_machine}"
    RESULT_VARIABLE elf_result
    OUTPUT_VARIABLE elf_output
    ERROR_VARIABLE elf_error)
  if(NOT elf_result EQUAL 0)
    message(
      FATAL_ERROR
        "vendor library architecture check failed: ${elf_error}${elf_output}")
  endif()

  get_filename_component(vendor_dir "${vendor}" DIRECTORY)
  set(${out_vendor_path}
      "${vendor}"
      PARENT_SCOPE)
  set(${out_vendor_dir}
      "${vendor_dir}"
      PARENT_SCOPE)
endfunction()

function(irmv_make_real_backend_available)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  irmv_require_real_dependency(vendor_path vendor_dir)

  # The SDK owns installation: the upstream package rules are disabled so an
  # embedded checkout cannot install a half-populated package or write a build
  # script into the dependency source tree.
  set(ENABLE_INSTALL
      OFF
      CACHE BOOL "" FORCE)
  set(ENABLE_BUILD_SCRIPT
      OFF
      CACHE BOOL "" FORCE)
  set(ENABLE_PYBINDING
      OFF
      CACHE BOOL "" FORCE)
  set(ENABLE_EXAMPLE
      OFF
      CACHE BOOL "" FORCE)
  set(ENABLE_TEST
      OFF
      CACHE BOOL "" FORCE)
  add_subdirectory("${IRMV_IRMV_CONTROL_SOURCE_DIR}"
                   "${CMAKE_BINARY_DIR}/irmv_control")
  # The embedded IMU project inherits an empty rpath item from its own package
  # configuration. Pin the build-tree rpath to the one directory that holds
  # libserial, so the installed dependency scan never sees an empty entry.
  if(TARGET yis_imu_driver)
    set_target_properties(
      yis_imu_driver PROPERTIES
      SKIP_BUILD_RPATH TRUE
      INSTALL_RPATH "$ORIGIN")
    set_target_properties(serial PROPERTIES INSTALL_RPATH "$ORIGIN")
    target_link_options(
      yis_imu_driver PRIVATE "-Wl,-rpath,${CMAKE_BINARY_DIR}/yis_imu")
  endif()
  set(IRMV_HAS_REAL_BACKEND
      TRUE
      PARENT_SCOPE)
  set(IRMV_VENDOR_LIBRARY
      "${vendor_path}"
      PARENT_SCOPE)
  set(IRMV_VENDOR_LIBRARY_DIR
      "${vendor_dir}"
      PARENT_SCOPE)
endfunction()
