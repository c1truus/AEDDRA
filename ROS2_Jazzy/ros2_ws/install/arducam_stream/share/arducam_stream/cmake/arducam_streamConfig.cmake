# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_arducam_stream_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED arducam_stream_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(arducam_stream_FOUND FALSE)
  elseif(NOT arducam_stream_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(arducam_stream_FOUND FALSE)
  endif()
  return()
endif()
set(_arducam_stream_CONFIG_INCLUDED TRUE)

# output package information
if(NOT arducam_stream_FIND_QUIETLY)
  message(STATUS "Found arducam_stream: 0.0.0 (${arducam_stream_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'arducam_stream' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT arducam_stream_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(arducam_stream_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${arducam_stream_DIR}/${_extra}")
endforeach()
