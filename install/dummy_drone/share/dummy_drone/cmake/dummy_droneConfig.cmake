# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dummy_drone_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dummy_drone_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dummy_drone_FOUND FALSE)
  elseif(NOT dummy_drone_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dummy_drone_FOUND FALSE)
  endif()
  return()
endif()
set(_dummy_drone_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dummy_drone_FIND_QUIETLY)
  message(STATUS "Found dummy_drone: 0.0.0 (${dummy_drone_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dummy_drone' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dummy_drone_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dummy_drone_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dummy_drone_DIR}/${_extra}")
endforeach()
