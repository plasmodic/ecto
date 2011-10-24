cmake_minimum_required(VERSION 2.8)
project(ecto)
set(${PROJECT_NAME}_GITHUB_URL https://github.com/plasmodic/ecto)

option(ECTO_TRACE_EXCEPTIONS "Trace ecto exceptions to stdout" OFF)

option(ECTO_STRESS_TEST "Stress testing of ecto" OFF)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  option(ECTO_LOGGING "Ecto logging" ON)
else()
  option(ECTO_LOGGING "Ecto logging" OFF)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH ${ecto_SOURCE_DIR}/cmake)

include(cmake/buildtype.cmake)
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

add_executable(msm msm.cpp)
add_executable(atomic atomic.cpp)
