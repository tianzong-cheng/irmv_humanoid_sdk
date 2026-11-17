include(FetchContent)

# The CycloneDDS C runtime and its C++ binding are a matched pair: 11.0.1
# changed both the API revision and the C/C++ shared-library SOVERSION (0 ->
# 11). Never pair one of them with a library built from other sources.
set(IRMV_CYCLONEDDS_VERSION "11.0.1")
set(IRMV_CYCLONEDDS_COMMIT
    "e54e991f75a3e67f8e628da3171122e36ea5b872")
set(IRMV_CYCLONEDDS_CXX_COMMIT
    "20ccaa519ca09a7b1776b8630389aa19ce8da61a")
set(IRMV_NLOHMANN_JSON_COMMIT
    "9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03")

set(IRMV_NLOHMANN_JSON_SOURCE_DIR
    ""
    CACHE PATH "Optional local nlohmann/json source checkout at the pinned commit")
set(IRMV_CYCLONEDDS_SOURCE_DIR
    ""
    CACHE PATH "Optional local CycloneDDS source checkout at the pinned commit")
set(IRMV_CYCLONEDDS_CXX_SOURCE_DIR
    ""
    CACHE PATH "Optional local cyclonedds-cxx source checkout at the pinned commit")
set(IRMV_CYCLONEDDS_PREFIX
    ""
    CACHE PATH "Optional prebuilt CycloneDDS 11.0.1 prefix")

# Provenance file carried by a prefix this file produced: the pinned release,
# commits and the checkouts the two-stage build actually used. A prefix whose
# manifest does not match the pins is refused instead of being rebuilt or used
# silently.
set(IRMV_CYCLONEDDS_MANIFEST "irmv-cyclonedds-manifest.txt")

function(irmv_declare_dependencies)
  FetchContent_Declare(
    cyclonedds
    GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
    GIT_TAG "${IRMV_CYCLONEDDS_COMMIT}"
    GIT_SHALLOW FALSE
  )
  FetchContent_Declare(
    cyclonedds_cxx
    GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
    GIT_TAG "${IRMV_CYCLONEDDS_CXX_COMMIT}"
    GIT_SHALLOW FALSE
  )
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG "${IRMV_NLOHMANN_JSON_COMMIT}"
    GIT_SHALLOW FALSE
  )
endfunction()

function(irmv_make_nlohmann_json_available)
  if(IRMV_NLOHMANN_JSON_SOURCE_DIR)
    execute_process(
      COMMAND git -C "${IRMV_NLOHMANN_JSON_SOURCE_DIR}" rev-parse HEAD
      RESULT_VARIABLE git_result
      OUTPUT_VARIABLE source_commit
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(NOT git_result EQUAL 0 OR NOT source_commit STREQUAL IRMV_NLOHMANN_JSON_COMMIT)
      message(
        FATAL_ERROR
          "IRMV_NLOHMANN_JSON_SOURCE_DIR must be a git checkout at ${IRMV_NLOHMANN_JSON_COMMIT}"
      )
    endif()
    set(FETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON
        "${IRMV_NLOHMANN_JSON_SOURCE_DIR}")
  endif()

  irmv_declare_dependencies()
  FetchContent_MakeAvailable(nlohmann_json)
endfunction()

# Fails unless <directory> is a git checkout of exactly <commit>. <option> is
# the cache variable being validated and only appears in the error message.
function(irmv_require_pinned_checkout option directory commit)
  execute_process(
    COMMAND git -C "${directory}" rev-parse HEAD
    RESULT_VARIABLE git_result
    OUTPUT_VARIABLE source_commit
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(NOT git_result EQUAL 0 OR NOT source_commit STREQUAL commit)
    message(FATAL_ERROR "${option} must be a git checkout at ${commit}")
  endif()
endfunction()

# Reads the provenance manifest of <prefix>. <out_ok> is TRUE only when the
# manifest exists, names this manifest format and pins the release and both
# dependency commits. A 0.10.5 manifest therefore fails every check that
# matters instead of being accepted as a compatible prefix.
function(irmv_verify_cyclonedds_manifest prefix out_ok out_reason)
  set(manifest "${prefix}/${IRMV_CYCLONEDDS_MANIFEST}")
  if(NOT EXISTS "${manifest}")
    set(${out_ok}
        FALSE
        PARENT_SCOPE)
    set(${out_reason}
        "no ${IRMV_CYCLONEDDS_MANIFEST}"
        PARENT_SCOPE)
    return()
  endif()

  file(STRINGS "${manifest}" lines)
  set(format_ok FALSE)
  set(version_ok FALSE)
  set(cyclonedds_ok FALSE)
  set(cyclonedds_cxx_ok FALSE)
  foreach(line IN LISTS lines)
    if(line STREQUAL "irmv-cyclonedds-manifest 2")
      set(format_ok TRUE)
    elseif(line STREQUAL "cyclonedds-version=${IRMV_CYCLONEDDS_VERSION}")
      set(version_ok TRUE)
    elseif(line STREQUAL "cyclonedds=${IRMV_CYCLONEDDS_COMMIT}")
      set(cyclonedds_ok TRUE)
    elseif(line STREQUAL "cyclonedds-cxx=${IRMV_CYCLONEDDS_CXX_COMMIT}")
      set(cyclonedds_cxx_ok TRUE)
    endif()
  endforeach()

  if(format_ok AND version_ok AND cyclonedds_ok AND cyclonedds_cxx_ok)
    set(${out_ok}
        TRUE
        PARENT_SCOPE)
    set(${out_reason}
        ""
        PARENT_SCOPE)
  else()
    set(${out_ok}
        FALSE
        PARENT_SCOPE)
    set(${out_reason}
        "manifest does not pin CycloneDDS ${IRMV_CYCLONEDDS_VERSION}, ${IRMV_CYCLONEDDS_COMMIT} and ${IRMV_CYCLONEDDS_CXX_COMMIT}"
        PARENT_SCOPE)
  endif()
endfunction()

function(irmv_write_cyclonedds_manifest prefix cyclonedds_source cyclonedds_cxx_source)
  file(
    WRITE "${prefix}/${IRMV_CYCLONEDDS_MANIFEST}"
    "irmv-cyclonedds-manifest 2\n"
    "cyclonedds-version=${IRMV_CYCLONEDDS_VERSION}\n"
    "cyclonedds=${IRMV_CYCLONEDDS_COMMIT}\n"
    "cyclonedds-cxx=${IRMV_CYCLONEDDS_CXX_COMMIT}\n"
    "cyclonedds-source=${cyclonedds_source}\n"
    "cyclonedds-cxx-source=${cyclonedds_cxx_source}\n"
  )
endfunction()

# Fails unless <prefix> looks like an installed CycloneDDS 11.0.1 with its C++
# binding: both CMake packages and both shared libraries, plus a manifest that
# pins this release and the exact commits. A prefix without a manifest - for
# example one built from the previous 0.10.5 pins - is refused rather than
# used.
function(irmv_require_cyclonedds_prefix prefix)
  foreach(required IN ITEMS
          "lib/cmake/CycloneDDS/CycloneDDSConfig.cmake"
          "lib/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfig.cmake")
    if(NOT EXISTS "${prefix}/${required}")
      message(
        FATAL_ERROR
          "${prefix} is not a CycloneDDS prefix: ${required} is missing")
    endif()
  endforeach()

  file(GLOB ddsc_libraries "${prefix}/lib/libddsc.*")
  file(GLOB ddscxx_libraries "${prefix}/lib/libddscxx.*")
  if(NOT ddsc_libraries OR NOT ddscxx_libraries)
    message(
      FATAL_ERROR
        "${prefix} is not a CycloneDDS prefix: libddsc/libddscxx are missing")
  endif()

  irmv_verify_cyclonedds_manifest("${prefix}" manifest_ok manifest_reason)
  if(NOT manifest_ok)
    message(
      FATAL_ERROR
        "${prefix} is not a CycloneDDS ${IRMV_CYCLONEDDS_VERSION} prefix produced from the pinned sources: ${manifest_reason}"
    )
  endif()
endfunction()

# Builds and installs the pinned C runtime, then the pinned C++ binding, into
# one private prefix. The two stages have to be separate and ordered: the pinned
# revisions export CycloneDDS::idlc from the C install only, and the C++ stage
# needs the installed C package. Both stages are configured by a nested CMake,
# so this project's warning flags are not propagated into the dependencies.
function(irmv_build_cyclonedds_prefix prefix)
  if(IRMV_CYCLONEDDS_SOURCE_DIR)
    irmv_require_pinned_checkout(IRMV_CYCLONEDDS_SOURCE_DIR
                                 "${IRMV_CYCLONEDDS_SOURCE_DIR}"
                                 "${IRMV_CYCLONEDDS_COMMIT}")
    set(FETCHCONTENT_SOURCE_DIR_CYCLONEDDS
        "${IRMV_CYCLONEDDS_SOURCE_DIR}")
  endif()
  if(IRMV_CYCLONEDDS_CXX_SOURCE_DIR)
    irmv_require_pinned_checkout(IRMV_CYCLONEDDS_CXX_SOURCE_DIR
                                 "${IRMV_CYCLONEDDS_CXX_SOURCE_DIR}"
                                 "${IRMV_CYCLONEDDS_CXX_COMMIT}")
    set(FETCHCONTENT_SOURCE_DIR_CYCLONEDDS_CXX
        "${IRMV_CYCLONEDDS_CXX_SOURCE_DIR}")
  endif()

  irmv_declare_dependencies()
  FetchContent_GetProperties(cyclonedds)
  if(NOT cyclonedds_POPULATED)
    FetchContent_Populate(cyclonedds)
  endif()
  FetchContent_GetProperties(cyclonedds_cxx)
  if(NOT cyclonedds_cxx_POPULATED)
    FetchContent_Populate(cyclonedds_cxx)
  endif()

  include(ProcessorCount)
  ProcessorCount(irmv_jobs)
  if(irmv_jobs EQUAL 0)
    set(irmv_jobs 2)
  endif()

  message(STATUS "Building CycloneDDS into ${prefix} from "
                 "${cyclonedds_SOURCE_DIR} and ${cyclonedds_cxx_SOURCE_DIR}")
  foreach(stage IN ITEMS c cxx)
    set(build_dir "${CMAKE_BINARY_DIR}/cyclonedds-build/${stage}")
    if(stage STREQUAL "c")
      set(
        stage_options
        -DBUILD_IDLC=ON
        -DBUILD_TESTING=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_DDSPERF=OFF
        -DENABLE_SECURITY=OFF
        -DENABLE_ICEORYX=OFF)
      set(stage_source "${cyclonedds_SOURCE_DIR}")
    else()
      set(
        stage_options
        -DBUILD_IDLLIB=ON
        -DBUILD_TESTING=OFF
        -DBUILD_EXAMPLES=OFF
        -DENABLE_ICEORYX=OFF
        -DENABLE_LEGACY=OFF
        -DCMAKE_PREFIX_PATH=${prefix})
      set(stage_source "${cyclonedds_cxx_SOURCE_DIR}")
    endif()

    execute_process(
      COMMAND
        "${CMAKE_COMMAND}" -S "${stage_source}" -B "${build_dir}"
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${prefix}
        -DWERROR=OFF ${stage_options}
      RESULT_VARIABLE configure_result)
    if(NOT configure_result EQUAL 0)
      message(FATAL_ERROR "Configuring CycloneDDS stage ${stage} failed")
    endif()

    execute_process(
      COMMAND "${CMAKE_COMMAND}" --build "${build_dir}" --parallel
              "${irmv_jobs}"
      RESULT_VARIABLE build_result)
    if(NOT build_result EQUAL 0)
      message(FATAL_ERROR "Building CycloneDDS stage ${stage} failed")
    endif()

    execute_process(
      COMMAND "${CMAKE_COMMAND}" --install "${build_dir}"
      RESULT_VARIABLE install_result)
    if(NOT install_result EQUAL 0)
      message(FATAL_ERROR "Installing CycloneDDS stage ${stage} failed")
    endif()
  endforeach()

  irmv_write_cyclonedds_manifest("${prefix}" "${cyclonedds_SOURCE_DIR}"
                                 "${cyclonedds_cxx_SOURCE_DIR}")
endfunction()

# Makes CycloneDDS 11.0.1 and its C++ binding available: either from the
# explicit IRMV_CYCLONEDDS_PREFIX cache, or by building the pinned sources into
# ${CMAKE_BINARY_DIR}/cyclonedds-prefix. Never falls back to unpinned sources
# and never silently skips the dependency: a missing or mismatched install is a
# configuration error.
function(irmv_make_cyclonedds_available)
  if(IRMV_CYCLONEDDS_PREFIX)
    set(prefix "${IRMV_CYCLONEDDS_PREFIX}")
  else()
    set(prefix "${CMAKE_BINARY_DIR}/cyclonedds-prefix")
  endif()

  if(NOT IRMV_CYCLONEDDS_PREFIX
     AND NOT EXISTS "${prefix}/${IRMV_CYCLONEDDS_MANIFEST}")
    irmv_build_cyclonedds_prefix("${prefix}")
  endif()

  irmv_require_cyclonedds_prefix("${prefix}")
  list(APPEND CMAKE_PREFIX_PATH "${prefix}")

  # Both pinned packages install a ConfigVersion.cmake, so the version is
  # enforced exactly on the imported targets. The manifest above additionally
  # binds those packages to the pinned source commits.
  find_package(CycloneDDS ${IRMV_CYCLONEDDS_VERSION} EXACT REQUIRED)
  find_package(CycloneDDS-CXX ${IRMV_CYCLONEDDS_VERSION} EXACT REQUIRED)

  # CycloneDDS 11.0.1's installed idlc/Generate.cmake uses the imported
  # CycloneDDS::idlc target when BUILD_IDLC is set and otherwise falls back to
  # find_program(idlc), which would require this private prefix's bin directory
  # on PATH. Selecting the imported target keeps generation bound to the pinned
  # install. This only affects this project's own IDL rules; the nested
  # dependency build above already configures the C stage with BUILD_IDLC=ON.
  set(BUILD_IDLC
      ON
      PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# ONNX Runtime
# ---------------------------------------------------------------------------

# The application's policy loader. ONNX Runtime is an application-only
# dependency: the lower process must keep running without it, and the build
# verifies that with `ldd build/robot_runtime` after the fact. The release is
# pinned by version, by URL and by the SHA-256 of the published archive, and the
# extracted prefix carries a manifest of all three; a prefix that does not match
# the pins is refused rather than used or silently rebuilt.
set(IRMV_ONNXRUNTIME_VERSION
    "1.22.0"
    CACHE STRING "Pinned ONNX Runtime release version"
)
set(IRMV_ONNXRUNTIME_SHA256
    "8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3"
    CACHE STRING "SHA-256 of the pinned onnxruntime-linux-x64 archive"
)
set(IRMV_ONNXRUNTIME_PREFIX
    ""
    CACHE PATH "Optional prebuilt ONNX Runtime prefix of the pinned release"
)

function(irmv_onnxruntime_manifest version sha256 out_text)
  set(${out_text}
      "irmv-onnxruntime 1\nversion ${version}\narchive onnxruntime-linux-x64-${version}.tgz\nsha256 ${sha256}\n"
      PARENT_SCOPE
  )
endfunction()

# Fails unless <prefix> is an extracted prefix of the pinned release: the
# headers, the versioned shared library and a manifest that pins version and
# archive digest.
function(irmv_require_onnxruntime_prefix prefix)
  if(NOT EXISTS "${prefix}/include/onnxruntime_cxx_api.h")
    message(
      FATAL_ERROR
        "IRMV_ONNXRUNTIME_PREFIX ${prefix} has no include/onnxruntime_cxx_api.h"
    )
  endif()
  set(_library "${prefix}/lib/libonnxruntime.so.${IRMV_ONNXRUNTIME_VERSION}")
  if(NOT EXISTS "${_library}")
    message(FATAL_ERROR "IRMV_ONNXRUNTIME_PREFIX ${prefix} has no ${_library}")
  endif()
  set(_marker "${prefix}/irmv-onnxruntime-manifest.txt")
  if(NOT EXISTS "${_marker}")
    message(
      FATAL_ERROR
        "IRMV_ONNXRUNTIME_PREFIX ${prefix} carries no irmv-onnxruntime-manifest.txt, so its provenance is unknown"
    )
  endif()
  irmv_onnxruntime_manifest(
    "${IRMV_ONNXRUNTIME_VERSION}" "${IRMV_ONNXRUNTIME_SHA256}" _expected
  )
  file(READ "${_marker}" _manifest)
  if(NOT _manifest STREQUAL _expected)
    message(
      FATAL_ERROR
        "IRMV_ONNXRUNTIME_PREFIX ${prefix} was built from another release than the pinned one\n"
        "expected:\n${_expected}found:\n${_manifest}"
    )
  endif()
endfunction()

# Makes the pinned ONNX Runtime available as IRMV::onnxruntime: either from an
# explicit prebuilt prefix, or by downloading and verifying the published
# release archive into ${CMAKE_BINARY_DIR}/onnxruntime-prefix. It never falls
# back to an unpinned library and never silently skips the dependency.
function(irmv_make_onnxruntime_available)
  if(IRMV_ONNXRUNTIME_PREFIX)
    irmv_require_onnxruntime_prefix("${IRMV_ONNXRUNTIME_PREFIX}")
    set(_prefix "${IRMV_ONNXRUNTIME_PREFIX}")
  else()
    set(_root "${CMAKE_BINARY_DIR}/onnxruntime-prefix")
    set(_prefix
        "${_root}/onnxruntime-linux-x64-${IRMV_ONNXRUNTIME_VERSION}"
    )
    set(_marker "${_prefix}/irmv-onnxruntime-manifest.txt")
    set(_library "${_prefix}/lib/libonnxruntime.so.${IRMV_ONNXRUNTIME_VERSION}")
    set(_usable FALSE)
    if(EXISTS "${_marker}" AND EXISTS "${_library}" AND EXISTS
       "${_prefix}/include/onnxruntime_cxx_api.h"
    )
      irmv_onnxruntime_manifest(
        "${IRMV_ONNXRUNTIME_VERSION}" "${IRMV_ONNXRUNTIME_SHA256}" _expected
      )
      file(READ "${_marker}" _manifest)
      if(_manifest STREQUAL _expected)
        set(_usable TRUE)
      endif()
    endif()
    if(NOT _usable)
      set(_archive
          "${CMAKE_BINARY_DIR}/irmv-onnxruntime-${IRMV_ONNXRUNTIME_VERSION}.tgz"
      )
      set(_url
          "https://github.com/microsoft/onnxruntime/releases/download/v${IRMV_ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${IRMV_ONNXRUNTIME_VERSION}.tgz"
      )
      message(STATUS "irmv: fetching ONNX Runtime ${IRMV_ONNXRUNTIME_VERSION}")
      file(
        DOWNLOAD "${_url}" "${_archive}"
        EXPECTED_HASH "SHA256=${IRMV_ONNXRUNTIME_SHA256}"
        SHOW_PROGRESS
        STATUS _download_status
      )
      list(GET _download_status 0 _download_code)
      if(NOT _download_code EQUAL 0)
        list(GET _download_status 1 _download_message)
        message(
          FATAL_ERROR
            "could not download the pinned ONNX Runtime: ${_download_message}"
        )
      endif()
      # The released archive has no CMake project of its own and carries one
      # version directory; it is unpacked beside the marker and the version
      # directory becomes the prefix. STRIP_COMPONENTS is not used because this
      # project supports CMake 3.22, where file(ARCHIVE_EXTRACT) does not have
      # it yet.
      file(REMOVE_RECURSE "${_root}")
      file(MAKE_DIRECTORY "${_root}")
      file(ARCHIVE_EXTRACT INPUT "${_archive}" DESTINATION "${_root}")
      irmv_onnxruntime_manifest(
        "${IRMV_ONNXRUNTIME_VERSION}" "${IRMV_ONNXRUNTIME_SHA256}" _manifest_text
      )
      file(WRITE "${_marker}" "${_manifest_text}")
      message(STATUS "irmv: ONNX Runtime extracted into ${_prefix}")
    endif()
  endif()

  irmv_require_onnxruntime_prefix("${_prefix}")
  set(IRMV_ONNXRUNTIME_ROOT
      "${_prefix}"
      CACHE INTERNAL "Root of the pinned ONNX Runtime that was made available"
  )

  if(NOT TARGET IRMV::onnxruntime)
    add_library(IRMV::onnxruntime SHARED IMPORTED GLOBAL)
    set_target_properties(
      IRMV::onnxruntime
      PROPERTIES IMPORTED_LOCATION
                 "${_prefix}/lib/libonnxruntime.so.${IRMV_ONNXRUNTIME_VERSION}"
                 INTERFACE_INCLUDE_DIRECTORIES "${_prefix}/include"
    )
  endif()
endfunction()
