cmake_minimum_required(VERSION 2.8)
#put this before project so that it gets picked up.
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")

project(ecto)

set(ECTO_MAJOR_VERSION 0)
set(ECTO_MINOR_VERSION 1)
set(ECTO_PATCH_VERSION 0)
set(ECTO_SOVERSION ${ECTO_MAJOR_VERSION}.${ECTO_MINOR_VERSION})
set(ECTO_VERSION ${ECTO_MAJOR_VERSION}.${ECTO_MINOR_VERSION}.${ECTO_PATCH_VERSION})
SET(ECTO_CODE_NAME "mold spore (alpha)")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(include_prefix include/ecto-${ECTO_VERSION})
set(share_prefix share/ecto-${ECTO_VERSION})

configure_file(${CMAKE_SOURCE_DIR}/cmake/version.hpp.in ${CMAKE_BINARY_DIR}/include/ecto/version.hpp)
include_directories(${CMAKE_BINARY_DIR}/include)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

execute_process(COMMAND mkdir -p ${LIBRARY_OUTPUT_PATH}/ecto)

execute_process(COMMAND ln -sf ${CMAKE_SOURCE_DIR}/python/ecto/__init__.py
  WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}/ecto
  )

include(${CMAKE_SOURCE_DIR}/cmake/ectoMacros.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ectoFindDeps.cmake)

set(ANAL_FLAGS "")
option(USE_ANAL_FLAGS "Use extra flags that help catch programming errors and prevent a build up of warnings" ON)
if(USE_ANAL_FLAGS)
    if(UNIX)
        set(ANAL_FLAGS " -Werror -Wall -Wl,--no-undefined")
    elseif(WIN32)
        set(ANAL_FLAGS "")
    elseif(APPLE)
        set(ANAL_FLAGS "")
    endif()
endif()

add_definitions (${ANAL_FLAGS})

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/src
  )
  
configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.in 
  ${CMAKE_BINARY_DIR}/python_path.sh)
message(STATUS "To setup your python path for ecto you may source:"
  " ${CMAKE_BINARY_DIR}/python_path.sh")

set(ecto_PYTHONPATH ${LIBRARY_OUTPUT_PATH} ${CMAKE_SOURCE_DIR}/python)
set(ecto_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)
set(ecto_LIBRARIES ${LIBRARY_OUTPUT_PATH}/libecto.so.${ECTO_VERSION})
set(ECTO_CONFIG_PATH  ${CMAKE_SOURCE_DIR}/cmake)
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig-version.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig-version.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoMacros.cmake
  ${CMAKE_BINARY_DIR}/ectoMacros.cmake @ONLY)
  
set(ecto_LIBRARIES ecto)

add_subdirectory(python/ecto)
add_subdirectory(src)
add_subdirectory(cmake)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_DOC "Build documentation" ON)
if(BUILD_DOC)
  add_subdirectory(doc)
endif()

option(BUILD_SAMPLES "Build samples." ON)
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

message(STATUS "Building ecto version: ${ECTO_VERSION}")
message(STATUS "Building ecto code name: ${ECTO_CODE_NAME}")
string(TOUPPER "CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}" flags_) 
message(STATUS "flags: ${ANAL_FLAGS} ${${flags_}}")

