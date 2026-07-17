# FindMiLib.cmake — imports mi-lib libraries via their <lib>-config scripts.
#
# The mi-lib stack (zeda/zm/zeo/roki/...) installs no CMake or pkg-config
# metadata; each library ships a generated `<lib>-config` shell script whose
# `--cflags` output carries -I/-D/-L flags and whose `-l` output carries the
# transitive -l link line.
#
# Usage:
#   find_package(MiLib REQUIRED COMPONENTS roki roki-fd dzco liw)
#   target_link_libraries(tgt PUBLIC MiLib::roki MiLib::roki-fd ...)
#
# Cache variables:
#   MILIB_PREFIX — installation prefix to search first (default: $ENV{HOME}/usr)

set(MILIB_PREFIX "$ENV{HOME}/usr" CACHE PATH "mi-lib installation prefix")

set(MiLib_FOUND TRUE)

foreach(_milib_component IN LISTS MiLib_FIND_COMPONENTS)
  if(TARGET MiLib::${_milib_component})
    set(MiLib_${_milib_component}_FOUND TRUE)
    continue()
  endif()

  find_program(MILIB_${_milib_component}_CONFIG
    NAMES ${_milib_component}-config
    HINTS ${MILIB_PREFIX}/bin)

  if(NOT MILIB_${_milib_component}_CONFIG)
    set(MiLib_${_milib_component}_FOUND FALSE)
    set(MiLib_FOUND FALSE)
    if(MiLib_FIND_REQUIRED_${_milib_component} OR MiLib_FIND_REQUIRED)
      message(FATAL_ERROR
        "MiLib component '${_milib_component}' not found: no "
        "'${_milib_component}-config' in ${MILIB_PREFIX}/bin or on PATH.\n"
        "Build and install the mi-lib stack first:\n"
        "  ./tools/bootstrap_milib.sh\n"
        "or point -DMILIB_PREFIX=<prefix> at an existing installation.")
    endif()
    continue()
  endif()

  execute_process(COMMAND ${MILIB_${_milib_component}_CONFIG} --cflags
    OUTPUT_VARIABLE _milib_cflags
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _milib_rc_cflags)
  execute_process(COMMAND ${MILIB_${_milib_component}_CONFIG} -l
    OUTPUT_VARIABLE _milib_libs
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _milib_rc_libs)
  if(NOT _milib_rc_cflags EQUAL 0 OR NOT _milib_rc_libs EQUAL 0)
    message(FATAL_ERROR
      "MiLib: '${MILIB_${_milib_component}_CONFIG}' failed to execute")
  endif()

  separate_arguments(_milib_cflags_list UNIX_COMMAND "${_milib_cflags}")
  separate_arguments(_milib_libs_list UNIX_COMMAND "${_milib_libs}")

  set(_milib_includes "")
  set(_milib_defs "")
  set(_milib_linkdirs "")
  foreach(_flag IN LISTS _milib_cflags_list)
    if(_flag MATCHES "^-I(.+)")
      list(APPEND _milib_includes "${CMAKE_MATCH_1}")
    elseif(_flag MATCHES "^-D(.+)")
      list(APPEND _milib_defs "${CMAKE_MATCH_1}")
    elseif(_flag MATCHES "^-L(.+)")
      list(APPEND _milib_linkdirs "${CMAKE_MATCH_1}")
    endif()
  endforeach()

  add_library(MiLib::${_milib_component} INTERFACE IMPORTED)
  set_target_properties(MiLib::${_milib_component} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_milib_includes}"
    INTERFACE_COMPILE_DEFINITIONS "${_milib_defs}"
    INTERFACE_LINK_DIRECTORIES "${_milib_linkdirs}"
    INTERFACE_LINK_LIBRARIES "${_milib_libs_list}")
  set(MiLib_${_milib_component}_FOUND TRUE)

  if(NOT MiLib_FIND_QUIETLY)
    message(STATUS "MiLib: ${_milib_component} via ${MILIB_${_milib_component}_CONFIG}")
  endif()
endforeach()
