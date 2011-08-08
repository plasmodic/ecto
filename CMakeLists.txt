cmake_minimum_required(VERSION 2.8)
#put this before project so that it gets picked up.
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")
project(ecto)

set(ECTO_MAJOR_VERSION 0)
set(ECTO_MINOR_VERSION 1)
set(ECTO_PATCH_VERSION 0)
set(ECTO_SOVERSION ${ECTO_MAJOR_VERSION}.${ECTO_MINOR_VERSION})
set(ECTO_VERSION ${ECTO_MAJOR_VERSION}.${ECTO_MINOR_VERSION}.${ECTO_PATCH_VERSION})
set(ECTO_CODE_NAME "Amoeba (alpha)")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(include_prefix include/ecto-${ECTO_VERSION})
set(share_prefix share/ecto-${ECTO_VERSION})

configure_file(${ecto_SOURCE_DIR}/cmake/version.hpp.in ${CMAKE_BINARY_DIR}/include/ecto/version.hpp)
include_directories(${CMAKE_BINARY_DIR}/include)

set(CMAKE_MODULE_PATH ${ecto_SOURCE_DIR}/cmake)

execute_process(COMMAND mkdir -p ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto)

execute_process(COMMAND ln -sf ${ecto_SOURCE_DIR}/python/ecto/__init__.py
  WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto
  )

option(ECTO_LOG_STATS "Generate logs containing fine-grained per-cell execution timing information.  You probably don't want this."
  OFF)

if(ECTO_LOG_STATS)
  add_definitions(-DECTO_LOG_STATS=1)
endif()

include(${ecto_SOURCE_DIR}/cmake/ectoMacros.cmake)
include(${ecto_SOURCE_DIR}/cmake/ectoFindDeps.cmake)

#FIXME Hack for standalone build
if(NOT ecto_kitchen_SOURCE_DIR)
  add_subdirectory(kitchen/doc doc)
endif()

set(ANAL_FLAGS "")
option(USE_ANAL_FLAGS "Use extra flags that help catch programming errors and prevent a build up of warnings" ON)
if(USE_ANAL_FLAGS)
  if(UNIX)
    set(ANAL_FLAGS " -Werror -Wall -Wl,--no-undefined")
  elseif(WIN32)
    set(ANAL_FLAGS "/D _CRT_SECURE_NO_WARNINGS /D BOOST_HAS_STDINT_H /wd4099 /wd4101 /wd4996 /wd4251 /wd4305 /D _WIN32_WINNT=0x0501")
  elseif(APPLE)
    set(ANAL_FLAGS "")
  endif()
endif()

add_definitions (${ANAL_FLAGS})

include_directories(
  ${ecto_SOURCE_DIR}/include
  ${ecto_SOURCE_DIR}/src
  )

if(UNIX)
  configure_file(${PROJECT_SOURCE_DIR}/cmake/python_path.sh.in 
    ${PROJECT_BINARY_DIR}/python_path.sh @ONLY@)
  #copy the python_path.sh.user.in for users that are not installing ecto.
  file(COPY ${PROJECT_SOURCE_DIR}/cmake/python_path.sh.user.in DESTINATION ${CMAKE_BINARY_DIR})
  if (NOT ecto_kitchen_SOURCE_DIR)
    message(STATUS "To setup your python path for ecto you may source:"
      " ${CMAKE_BINARY_DIR}/python_path.sh")
  endif()
elseif(WIN32)
  configure_file(${ecto_SOURCE_DIR}/cmake/pathz.bat.in 
    ${CMAKE_BINARY_DIR}/pathz.bat @ONLY@)
  if(NOT ecto_kitchen_SOURCE_DIR)
    message(STATUS "To setup your pathz for ecto you should run:"
      "${CMAKE_BINARY_DIR}/pathz.bat")
  endif()
endif()




set(ecto_PYTHONPATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${ecto_SOURCE_DIR}/python)
set(ecto_INCLUDE_DIRS ${ecto_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)
if(UNIX)
  set(ecto_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libecto.so.${ECTO_VERSION})
elseif(WIN32)
  set(ecto_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto_cpp.dll)
endif()

set(ECTO_CONFIG_PATH  ${ecto_SOURCE_DIR}/cmake)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig-version.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig-version.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoMacros.cmake
  ${CMAKE_BINARY_DIR}/ectoMacros.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/rosbuild_lite.cmake
  ${CMAKE_BINARY_DIR}/rosbuild_lite.cmake @ONLY)

set(ecto_LIBRARIES ecto)

add_subdirectory(python)
add_subdirectory(src)
add_subdirectory(cmake)
add_subdirectory(doc/source/src doc_examples)
add_subdirectory(doc/source/overview doc_overview)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_SAMPLES "Build samples." ON)
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

message(STATUS "Building ecto version: ${ECTO_VERSION}")
message(STATUS "Building ecto code name: ${ECTO_CODE_NAME}")
string(TOUPPER "CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}" flags_) 
message(STATUS "flags: ${ANAL_FLAGS} ${${flags_}}")

