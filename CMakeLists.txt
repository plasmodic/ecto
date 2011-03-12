cmake_minimum_required(VERSION 2.8)

project(ecto)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Boost COMPONENTS
 program_options
 thread
 python
)

find_package(PythonLibs)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  )

#make sure undefined symbols are errors (helps a lot for python)!
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wl,--no-undefined")

macro(ectomodule NAME)

add_library(${NAME}_pymodule SHARED
  ${ARGN}
  )

set_target_properties(${NAME}_pymodule
  PROPERTIES
  OUTPUT_NAME ${NAME}
  PREFIX ""
#SUFFIX ".pyd"
  )

target_link_libraries(${NAME}_pymodule
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ecto
)
endmacro()

add_library(ecto SHARED
  src/connection.cpp
  src/module.cpp
  src/plasm.cpp
  src/util.cpp
)

ectomodule(_ecto
  src/ecto_py/ecto.cpp
  src/ecto_py/plasm.cpp
  src/ecto_py/connection.cpp
  src/ecto_py/module.cpp
)

set(BUILD_TEST_MODULES ON CACHE BOOL "Build test modules")

if(BUILD_TEST_MODULES)
  ectomodule(buster
    test/modules/buster.cpp
  )
endif()

  
