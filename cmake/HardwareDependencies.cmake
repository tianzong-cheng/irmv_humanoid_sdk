# Real hardware dependencies.
#
# The default build is fake-only: it neither fetches nor links the vendor CAN-FD
# library, the upstream runtime I/O or the IMU driver. Turning
# IRMV_ENABLE_REAL_BACKEND on makes those dependencies required and fails the
# configuration when their provenance cannot be checked, instead of silently
# producing a runtime that cannot reach a device.

set(IRMV_IRMV_CONTROL_SOURCE_DIR
    ""
    CACHE PATH "IRMVControl checkout that provides IRMVControl::runtime_io")

set(IRMV_IRMV_CONTROL_COMMIT
    ""
    CACHE STRING "Required IRMVControl commit (empty skips the check)")

set(IRMV_YIS_IMU_SOURCE_DIR
    ""
    CACHE PATH "Pinned yis_imu_driver checkout (empty builds without IMU support)")

function(irmv_require_real_dependency)
  if(NOT IRMV_IRMV_CONTROL_SOURCE_DIR)
    message(
      FATAL_ERROR
        "IRMV_ENABLE_REAL_BACKEND=ON requires IRMV_IRMV_CONTROL_SOURCE_DIR")
  endif()
  if(NOT EXISTS "${IRMV_IRMV_CONTROL_SOURCE_DIR}/include/hardware/runtime_io.h")
    message(
      FATAL_ERROR
        "${IRMV_IRMV_CONTROL_SOURCE_DIR} does not provide include/hardware/runtime_io.h"
    )
  endif()
  if(IRMV_IRMV_CONTROL_COMMIT)
    execute_process(
      COMMAND git -C "${IRMV_IRMV_CONTROL_SOURCE_DIR}" rev-parse HEAD
      RESULT_VARIABLE git_result
      OUTPUT_VARIABLE actual_commit
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(NOT git_result EQUAL 0 OR NOT actual_commit STREQUAL
       IRMV_IRMV_CONTROL_COMMIT)
      message(
        FATAL_ERROR
          "IRMVControl checkout is not the pinned commit ${IRMV_IRMV_CONTROL_COMMIT}"
      )
    endif()
  endif()
endfunction()

function(irmv_make_real_backend_available)
  irmv_require_real_dependency()
  set(ENABLE_EXAMPLE
      OFF
      CACHE BOOL "" FORCE)
  set(ENABLE_TEST
      OFF
      CACHE BOOL "" FORCE)
  add_subdirectory("${IRMV_IRMV_CONTROL_SOURCE_DIR}"
                   "${CMAKE_BINARY_DIR}/irmv_control")
  set(IRMV_HAS_REAL_BACKEND
      TRUE
      PARENT_SCOPE)
endfunction()
