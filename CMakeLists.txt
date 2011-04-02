cmake_minimum_required(VERSION 2.8)
project(ecto)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(${CMAKE_SOURCE_DIR}/cmake/ectoMacros.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ectoFindDeps.cmake)

option(ECTO_BUILD_UI "build the ui stuffs" OFF)
if (ECTO_BUILD_UI)
  add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/python/ecto/ui/uic.py
    COMMAND pyside-uic ${CMAKE_SOURCE_DIR}/python/ecto/ui/ecto.ui > ${CMAKE_SOURCE_DIR}/python/ecto/ui/uic.py
    DEPENDS ${CMAKE_SOURCE_DIR}/python/ecto/ui/ecto.ui
    COMMENT "Generating python ui code with pyside-uic")

  add_custom_target(uic ALL
    DEPENDS ${CMAKE_SOURCE_DIR}/python/ecto/ui/uic.py
    )

  configure_file(${CMAKE_SOURCE_DIR}/python/ectoedit.py.in
    ${CMAKE_BINARY_DIR}/bin/ectoedit.py
    @ONLY)
endif()

#openni makes this fail -Werror
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wl,--no-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wl,--no-undefined")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBOOST_DISABLE_ASSERTS")

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  )
  
add_library(ecto SHARED
  src/tendril.cpp
  src/module.cpp
  src/plasm.cpp
  src/util.cpp
  )
  
target_link_libraries(ecto
  ${ECTO_DEP_LIBS}
  )

ectomodule(ecto
  src/ecto_py/ecto.cpp
  src/ecto_py/plasm.cpp
  src/ecto_py/tendril.cpp
  src/ecto_py/module.cpp
  )

configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.in 
  ${CMAKE_BINARY_DIR}/python_path.sh)
message(STATUS "To setup your python path for ecto you may source:"
  " ${CMAKE_BINARY_DIR}/python_path.sh")

configure_file(python/ecto/__init__.py 
  ${CMAKE_BINARY_DIR}/lib/ecto/__init__.py
  @ONLY)

set(ecto_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${LIBRARY_OUTPUT_PATH})

configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)

add_subdirectory(test)

#add_subdirectory(samples)


