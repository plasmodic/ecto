cmake_minimum_required(VERSION 2.8)
project(ecto)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

execute_process(COMMAND mkdir -p ${LIBRARY_OUTPUT_PATH}/ecto)

execute_process(COMMAND ln -sf ${CMAKE_SOURCE_DIR}/python/ecto/__init__.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/lib/ecto
  )

include(${CMAKE_SOURCE_DIR}/cmake/ectoMacros.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ectoFindDeps.cmake)

#set flags so that we have awesome compile time errors on all warnings
#notice the -Wl,--no-undefined helps with making sure all symbols are found during
#link time.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wl,--no-undefined")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBOOST_DISABLE_ASSERTS")

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/src
  )
  
configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.in 
  ${CMAKE_BINARY_DIR}/python_path.sh)
message(STATUS "To setup your python path for ecto you may source:"
  " ${CMAKE_BINARY_DIR}/python_path.sh")

set(ecto_PYTHONPATH ${LIBRARY_OUTPUT_PATH} ${CMAKE_SOURCE_DIR}/python)

set(ecto_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${LIBRARY_OUTPUT_PATH})
 
set(ECTO_CONFIG_PATH  ${CMAKE_SOURCE_DIR}/cmake)

configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)



include(cmake/ectoInstall.cmake)

include_directories(
  # lockfree contains atomic, include the bryce/hartmut version first
  ${CMAKE_SOURCE_DIR}/boost.atomic
  ${CMAKE_SOURCE_DIR}/boost.lockfree 
  )


add_subdirectory(src)


option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  add_subdirectory(test)
  ENABLE_TESTING()
  configure_file(${CMAKE_SOURCE_DIR}/test/run_nose.py.in ${CMAKE_BINARY_DIR}/run_nose.py @ONLY)
  add_test(NAME nosetests
    COMMAND "/usr/bin/python" 
    ${CMAKE_BINARY_DIR}/run_nose.py -v -w ${CMAKE_SOURCE_DIR}/test/python)
endif()

option(BUILD_DOC "Build documentation" ON)

if(BUILD_DOC)
  add_subdirectory(doc)
endif()

option(BUILD_SAMPLES "Build samples." ON)
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

add_executable(rpattern rpattern.cpp)

