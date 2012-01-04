# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
#rosbuild_lite.cmake
#
#   A set of cmake macros that allow non rosbuild projects to use ros packages
#
#   rosbuild_lite_init()
#       This finds ROSPACK_EXECUTABLE, and ROSMSG_EXECUTABLE, along with setting
#       ROS_ROOT and ROS_PACKAGE_PATH. You may change the ros distro
#       by setting ROS_ROOT and ROS_PACKAGE_PATH in the cmake cache.
#       ROS_FOUND will be set to TRUE if this is successful, and will be unset
#       otherwise.
#
#
#   rospack( VAR COMMAND PACKAGE )
#       This calls rospack and stores the result in VAR.  It uses the COMMAND
#       as the rospack subcommand, and PACKAGE as the ros package.  The ROS_PACKAGE_PATH
#       that is set in the cache must contain the ros package you wish to build against.
#
#   find_ros_package( PACKAGE )
#       Finds a ROS package, by the name PACKAGE, it must be in the cached ROS_PACKAGE_PATH.
#       
option(ROS_CONFIGURE_VERBOSE OFF)

macro( rosbuild_lite_init )
  set(ROS_ROOT_DEFAULT /opt/ros/electric/ros CACHE PATH "The default ROS_ROOT")
  set(ROS_PACKAGE_PATH_DEFAULT /opt/ros/electric/stacks CACHE PATH "The default ROS_PACKAGE_PATH")

  if (NOT ROS_ROOT)
    if ("$ENV{ROS_ROOT}" STREQUAL "")
      message(STATUS "*** ROS_ROOT is not set... is your environment set correctly? Setting to default: ${ROS_ROOT_DEFAULT}")
      set(ROS_ROOT ${ROS_ROOT_DEFAULT} CACHE PATH  "ROS_ROOT path")
    else()
      set(ROS_ROOT "$ENV{ROS_ROOT}" CACHE PATH  "ROS_ROOT path")
    endif()
  endif()

  if (NOT ROS_PACKAGE_PATH)
    if ("$ENV{ROS_PACKAGE_PATH}" STREQUAL "")
      message(STATUS "*** ROS_PACKAGE_PATH is not set... is your environment set correctly? Setting to default: ${ROS_PACKAGE_PATH_DEFAULT}")
      set(ROS_PACKAGE_PATH ${ROS_PACKAGE_PATH_DEFAULT} CACHE PATH "ROS_PACKAGE_PATH path")
    else()
      set(ROS_PACKAGE_PATH "$ENV{ROS_PACKAGE_PATH}" CACHE PATH "ROS_PACKAGE_PATH path")
    endif()
  endif()

  find_program(ROSPACK_EXECUTABLE rospack PATHS ${ROS_ROOT}/bin DOC "the rospack executable.")
  find_program(ROSMSG_EXECUTABLE rosmsg PATHS ${ROS_ROOT}/bin DOC "rosmsg executable")

  if (ROSPACK_EXECUTABLE)
    set(ROS_FOUND TRUE)
    message(STATUS "*** ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}")
    message(STATUS "*** ROSPACK_EXECUTABLE=${ROSPACK_EXECUTABLE}")
  else()
    unset(ROS_FOUND)
  endif()
endmacro()

#attempts to set ENV variables so that ROS commands will work.
#This appears to work well on linux, but may be questionable on
#other platforms.
macro (_set_ros_env)
  set(ORIG_ROS_ROOT $ENV{ROS_ROOT})
  set(ORIG_ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
  set(ORIG_PATH $ENV{PATH})
  set(ORIG_PYTHONPATH $ENV{PYTHONPATH})
  set(ENV{ROS_ROOT} ${ROS_ROOT})
  set(ENV{ROS_PACKAGE_PATH} ${ROS_PACKAGE_PATH})
  set(ENV{PATH} "${ROS_ROOT}/bin:$ENV{PATH}")
  set(ENV{PYTHONPATH} "${ROS_ROOT}/core/roslib/src:$ENV{PYTHONPATH}")
endmacro()

#unset environment
macro (_unset_ros_env)
  set(ENV{ROS_ROOT} ${ORIG_ROS_ROOT})
  set(ENV{ROS_PACKAGE_PATH} ${ORIG_ROS_PACKAGE_PATH})
  set(ENV{PATH} "${ORIG_PATH}")
  set(ENV{PYTHONPATH} "${ORIG_PYTHONPATH}")
endmacro()

macro (rospack VAR COMMAND PACKAGE)
  set(cachevar ROSPACK_${PACKAGE}_${COMMAND})
  set(${cachevar} "not-run-yet-NOTFOUND" CACHE INTERNAL "")
  if(NOT ${cachevar})
    if (NOT ROSPACK_EXECUTABLE)
      message(WARNING "*** rosmsg There is no rosmsg executable!")
      set(rospack_error "ROSPACK_EXECUTABLE not found") 
    else()
      _set_ros_env()
      execute_process(COMMAND ${ROSPACK_EXECUTABLE} ${COMMAND} ${PACKAGE}
        OUTPUT_VARIABLE ROSPACK_OUT
        ERROR_VARIABLE rospack_error
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE rospack_error_code
        )
      _unset_ros_env()
      if(${rospack_error_code} EQUAL 0)
        unset(rospack_error)
        unset(rospack_error_code)
      endif()
    endif()
    if(rospack_error)
      message(STATUS "***")
      message(STATUS "*** rospack ${COMMAND} ${PACKAGE} failed: ${rospack_error}")
      message(STATUS "***")
      set(${cachevar} "ROSPACK_${PACKAGE}_${COMMAND}-NOTFOUND"
        CACHE INTERNAL "rospack output for rospack ${PACKAGE} ${COMMAND}")
    else()
      separate_arguments(ROSPACK_SEPARATED UNIX_COMMAND ${ROSPACK_OUT})
      set(${cachevar} ${ROSPACK_SEPARATED} CACHE INTERNAL "value")
      set(${VAR} ${ROSPACK_SEPARATED} CACHE INTERNAL "" FORCE)
      # message("${VAR} == ${${VAR}}")
    endif()
  else()
    set(${VAR} "${${cachevar}}")
  endif()
endmacro()

macro (find_ros_package PACKAGE)
  if (NOT ${PACKAGE}_DIR)
    rospack(${PACKAGE}_DIR find ${PACKAGE})
  endif()

  if(NOT ${PACKAGE}_DIR)
    message(STATUS "Could not find package ${PACKAGE} via rosmake")
  else()
    if(NOT ${PACKAGE}_FOUND)
      message(STATUS "Finding ROS package: ${PACKAGE}")
      rospack(${PACKAGE}_INCLUDE_DIRS cflags-only-I ${PACKAGE})
      rospack(${PACKAGE}_DEFINITIONS cflags-only-other ${PACKAGE})

      rospack(libdirs libs-only-L ${PACKAGE})
      rospack(libnames libs-only-l ${PACKAGE})

      set(${PACKAGE}_LIBRARIES "" CACHE INTERNAL "")

      foreach(libname ${ROSPACK_${PACKAGE}_libs-only-l})
        find_library(${libname}_LIBRARY
          NAMES ${libname}
          PATHS ${ROSPACK_${PACKAGE}_libs-only-L}
          NO_DEFAULT_PATH
          )
        find_library(${libname}_LIBRARY ${libname})
        # message("${libname}_LIBRARY ${${libname}_LIBRARY}")
        if (NOT ${libname}_LIBRARY)
          message(FATAL_ERROR "uh oh ${PACKAGE} ${libname} found us ${thelib}")
        endif()
        set(${PACKAGE}_LIBRARIES ${${PACKAGE}_LIBRARIES};${${libname}_LIBRARY})
        mark_as_advanced(${libname}_LIBRARY)
      endforeach()
      set(${PACKAGE}_LIBRARIES ${${PACKAGE}_LIBRARIES} CACHE INTERNAL "" FORCE)
    endif()

    include_directories(${${PACKAGE}_INCLUDE_DIRS})
    add_definitions(${${PACKAGE}_DEFINITIONS})

  endif() # not PACKAGE_DIR

  if (${PACKAGE}_DIR)

    # message("${PACKAGE}_LIBRARIES ${${PACKAGE}_LIBRARIES}")
    list(LENGTH ${PACKAGE}_LIBRARIES nlibs)
    list(LENGTH ${PACKAGE}_INCLUDE_DIRS nincludes)
    list(LENGTH ${PACKAGE}_DEFINITIONS ndefs)
    if(ROS_CONFIGURE_VERBOSE)
        message(STATUS "+ ${PACKAGE} at ${${PACKAGE}_DIR}")
        message(STATUS "+   ${nlibs} libraries, ${nincludes} include directories, ${ndefs} compile definitions")
    endif()
    set(${PACKAGE}_FOUND TRUE CACHE INTERNAL "" FORCE)
  else()
    message(WARNING "+ ${PACKAGE}: NOT FOUND")
    set(${PACKAGE}_FOUND FALSE CACHE INTERNAL "" FORCE)
  endif()

endmacro()

macro(rosmsg VAR CMD PACKAGE)
  if (NOT ROSMSG_EXECUTABLE)
    set(rosmsg_error "rosmsg There is no rosmsg executable!")
  else()
    _set_ros_env()
    execute_process(COMMAND ${ROSMSG_EXECUTABLE} ${CMD} ${PACKAGE}
      OUTPUT_VARIABLE ROSMSG_OUT
      ERROR_VARIABLE rosmsg_error
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE rosmsg_error_code
      )
    _unset_ros_env()
    if(${rosmsg_error_code} EQUAL 0)
      unset(rosmsg_error)
      unset(rosmsg_error_code)
    endif()
  endif()
  if (rosmsg_error)
    message(WARNING "*** rosmsg ${CMD} ${PACKAGE} failed: ${rosmsg_error}")
  else()
    separate_arguments(ROSPACK_SEPARATED UNIX_COMMAND ${ROSMSG_OUT})
    set(${VAR} ${ROSPACK_SEPARATED} CACHE INTERNAL "" FORCE)
    message(STATUS "rosmsg ${VAR} == ${${VAR}}")
  endif()
endmacro()

