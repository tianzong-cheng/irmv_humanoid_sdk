include(FetchContent)

set(IRMV_CYCLONEDDS_COMMIT
    "2cdd114cbd18340c606573b4cc8dc20cc161ec5a")
set(IRMV_CYCLONEDDS_CXX_COMMIT
    "50984cfc9a92d1ae1903d7b1479a110c77fba240")
set(IRMV_NLOHMANN_JSON_COMMIT
    "9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03")

set(IRMV_NLOHMANN_JSON_SOURCE_DIR
    ""
    CACHE PATH "Optional local nlohmann/json source checkout at the pinned commit")

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
