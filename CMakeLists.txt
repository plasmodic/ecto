cmake_minimum_required(VERSION 2.8)
project(ecto)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH ${ecto_SOURCE_DIR}/cmake)

include(cmake/buildtype.cmake)
include(cmake/version.cmake)
include(cmake/fastidious.cmake)
include(cmake/git.cmake)
include(cmake/ectoMacros.cmake)
include(cmake/ectoFindDeps.cmake)
include(cmake/config.cmake)

git_status(ecto)
ecto_python_env_gen()

include_directories(include)
add_subdirectory(src)
add_subdirectory(cmake)
add_subdirectory(python)
add_subdirectory(doc)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_SAMPLES "Build samples." ON)
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

option(BUILD_doc_errata "Build doc errata" ON)
if(BUILD_doc_errata)
  add_subdirectory(doc/source doc_errata)
endif()

