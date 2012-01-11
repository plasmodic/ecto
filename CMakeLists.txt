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
cmake_minimum_required(VERSION 2.8)
project(ecto)
find_package(catkin REQUIRED)
catkin_stack(ecto)

set(${PROJECT_NAME}_GITHUB_URL https://github.com/plasmodic/ecto)

option(ECTO_TRACE_EXCEPTIONS "Trace ecto exceptions to stdout" OFF)

option(ECTO_STRESS_TEST "Stress testing of ecto" OFF)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  option(ECTO_LOGGING "Ecto logging" ON)
else()
  option(ECTO_LOGGING "Ecto logging" OFF)
endif()

option(ECTO_WITH_INSTRUMENTATION "Instrument schedulers (for debugging)" OFF)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
list(APPEND CMAKE_MODULE_PATH ${ecto_SOURCE_DIR}/cmake)

#include(cmake/buildtype.cmake)
include(cmake/version.cmake)
include(cmake/git.cmake)
include(cmake/ectoMacros.cmake)
include(cmake/boost.cmake)
include(cmake/ectoFindDeps.cmake)
include(cmake/config.cmake)
include(cmake/fastidious.cmake)
include(cmake/doc.cmake)

git_status(ecto)
ecto_python_env_gen()

include_directories(include)
include_directories(src/lib)
add_subdirectory(src)
add_subdirectory(cmake)
add_subdirectory(python)

option(BUILD_DOC "Build docs, this will build tests and samples as well." ON)
if(BUILD_DOC)
  set(BUILD_TEST ON CACHE BOOL "Tests must be built if docs are expected to build." FORCE)
  set(BUILD_SAMPLES ON CACHE BOOL "Samples must be built if docs are expected to build." FORCE)
  add_subdirectory(doc)
endif()

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_SAMPLES "Build samples." ON)
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

catkin_project(ecto
 INCLUDE_DIRS
 include
 ${ecto_BINARY_DIR}/include
 LIBRARIES ecto
 CFG_EXTRAS ecto-extras.cmake
)

catkin_add_env_hooks(10ecto SHELLS sh)
