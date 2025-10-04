# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imx_stream_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imx_stream_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imx_stream_FOUND FALSE)
  elseif(NOT imx_stream_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imx_stream_FOUND FALSE)
  endif()
  return()
endif()
set(_imx_stream_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imx_stream_FIND_QUIETLY)
  message(STATUS "Found imx_stream: 0.0.0 (${imx_stream_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imx_stream' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT imx_stream_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imx_stream_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imx_stream_DIR}/${_extra}")
endforeach()
