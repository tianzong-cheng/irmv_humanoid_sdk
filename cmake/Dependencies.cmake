include(FetchContent)

set(IRMV_CYCLONEDDS_COMMIT
    "2cdd114cbd18340c606573b4cc8dc20cc161ec5a")
set(IRMV_CYCLONEDDS_CXX_COMMIT
    "50984cfc9a92d1ae1903d7b1479a110c77fba240")
set(IRMV_NLOHMANN_JSON_COMMIT
    "9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03")

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
