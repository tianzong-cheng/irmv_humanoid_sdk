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
if(TARGET irmv_maintenance)
  install(TARGETS irmv_maintenance
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()
if(IRMV_HAS_REAL_BACKEND)
  if(TARGET controlcanfd)
    get_target_property(_irmv_controlcanfd_location controlcanfd
                        IMPORTED_LOCATION)
    if(NOT _irmv_controlcanfd_location)
      message(FATAL_ERROR "controlcanfd has no IMPORTED_LOCATION")
    endif()
    install(FILES "${_irmv_controlcanfd_location}"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}")
  endif()
  if(TARGET yis_imu_driver)
    install(TARGETS yis_imu_driver serial
            LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")
  endif()
endif()
if(TARGET application)
  install(TARGETS application RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()
if(TARGET external_producer)
  install(TARGETS external_producer RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

# The one-line entry point. It is configured rather than copied, because the
# default deployment file, the bound simulation interpreter and the compiled
# Real capability are constants of this build: a moved prefix still points at
# its own etc/irmv, and a Real build never silently starts the shipped Fake
# deployment. The development default is the shipped sdk.json; the sim and robot
# presets point at etc/irmv/deployment-sdk.json, which the operator provisions
# explicitly and which is not installed as a sample. The path is relative to the
# installed bin directory, so it survives copying the prefix.
if(IRMV_ENABLE_REAL_BACKEND)
  set(_irmv_default_launch_config_name "deployment-sdk.json")
else()
  set(_irmv_default_launch_config_name "sdk.json")
endif()
file(
  RELATIVE_PATH _irmv_default_launch_config
  "${CMAKE_INSTALL_FULL_BINDIR}"
  "${CMAKE_INSTALL_FULL_SYSCONFDIR}/irmv/${_irmv_default_launch_config_name}"
)
set(
  IRMV_LAUNCH_CONFIG "${_irmv_default_launch_config}"
  CACHE STRING
  "Default config of irmv-sdk, relative to the installed bin directory"
)
if(TARGET application)
  configure_file(
    "${PROJECT_SOURCE_DIR}/tools/irmv_sdk.py"
    "${CMAKE_CURRENT_BINARY_DIR}/irmv_sdk.py"
    @ONLY
  )
  install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/irmv_sdk.py"
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    RENAME irmv-sdk
  )
  install(
    PROGRAMS "${PROJECT_SOURCE_DIR}/tools/sdk_launch.py"
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
  )
endif()

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
if(TARGET application)
  set_target_properties(
    application
    PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()
if(TARGET irmv_maintenance)
  set_target_properties(
    irmv_maintenance
    PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()
# The example producer links the same pinned DDS libraries the application does
# (its transport), so the installed binary resolves them from <prefix>/lib
# exactly like the two processes and runs from a tree that was moved away from
# its build directory. The reference-sample tool links ONNX Runtime for the same
# reason the application does, and needs the same entry.
if(TARGET external_producer)
  set_target_properties(
    external_producer
    PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()
if(TARGET verify_policy_sample)
  set_target_properties(
    verify_policy_sample
    PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()

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
set(_irmv_install_executables "$<TARGET_FILE:robot_runtime>")
set(_irmv_install_directories
    "$<TARGET_FILE_DIR:CycloneDDS-CXX::ddscxx>"
    "$<TARGET_FILE_DIR:CycloneDDS::ddsc>")
if(TARGET application)
  list(APPEND _irmv_install_executables "$<TARGET_FILE:application>")
  list(APPEND _irmv_install_directories "${IRMV_ONNXRUNTIME_ROOT}/lib")
endif()
if(TARGET irmv_maintenance)
  list(APPEND _irmv_install_executables "$<TARGET_FILE:irmv_maintenance>")
endif()
if(TARGET controlcanfd)
  get_target_property(_irmv_controlcanfd_location controlcanfd
                      IMPORTED_LOCATION)
  get_filename_component(_irmv_controlcanfd_dir
                         "${_irmv_controlcanfd_location}" DIRECTORY)
  list(APPEND _irmv_install_directories "${_irmv_controlcanfd_dir}")
endif()
if(TARGET yis_imu_driver)
  list(APPEND _irmv_install_directories
       "$<TARGET_FILE_DIR:yis_imu_driver>"
       "$<TARGET_FILE_DIR:serial>")
endif()
string(REPLACE ";" "\" \"" _irmv_install_executables_arg
               "${_irmv_install_executables}")
string(REPLACE ";" "\" \"" _irmv_install_directories_arg
               "${_irmv_install_directories}")
set(_irmv_install_executables_arg "\"${_irmv_install_executables_arg}\"")
set(_irmv_install_directories_arg "\"${_irmv_install_directories_arg}\"")
install(
  CODE "
set(_irmv_libdir \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\")
foreach(_irmv_bundled IN ITEMS libyis_imu_driver.so libserial.so)
  if(EXISTS \"\${_irmv_libdir}/\${_irmv_bundled}\")
    file(RPATH_SET FILE \"\${_irmv_libdir}/\${_irmv_bundled}\"
         NEW_RPATH \"\\$ORIGIN\")
  endif()
endforeach()
file(
  GET_RUNTIME_DEPENDENCIES
  EXECUTABLES ${_irmv_install_executables_arg}
  DIRECTORIES ${_irmv_install_directories_arg}
  RESOLVED_DEPENDENCIES_VAR _irmv_resolved
  UNRESOLVED_DEPENDENCIES_VAR _irmv_unresolved
  PRE_EXCLUDE_REGEXES
    \"^libc[.]so\" \"^libm[.]so\" \"^libdl[.]so\" \"^librt[.]so\"
    \"^libpthread[.]so\" \"^libgcc_s[.]so\" \"^libstdc[+][+]\" \"^libatomic[.]so\"
    \"^ld-linux\"
  POST_EXCLUDE_REGEXES
    \"^/lib/\" \"^/lib64/\" \"^/usr/lib/\" \"^/usr/lib64/\" \"^/usr/local/lib/\"
  POST_INCLUDE_REGEXES \"^libonnxruntime\" \"^libddsc\" \"^libcontrolcanfd\" \"^libyis_imu_driver\" \"^libserial\"
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
if(EXISTS \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/irmv_maintenance\")
  file(
    RPATH_CHECK
    FILE \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/irmv_maintenance\"
    RPATH \"\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}\"
  )
endif()
foreach(_irmv_optional_binary IN ITEMS application external_producer verify_policy_sample)
  if(EXISTS \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/\${_irmv_optional_binary}\")
    file(
      RPATH_CHECK
      FILE \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/\${_irmv_optional_binary}\"
      RPATH \"\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}\"
    )
  endif()
endforeach()
"
)
