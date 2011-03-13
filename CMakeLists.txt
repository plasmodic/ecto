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

add_library(${NAME}_ectomodule SHARED
  ${ARGN}
  )

set_target_properties(${NAME}_ectomodule
  PROPERTIES
  OUTPUT_NAME ${NAME}
  PREFIX ""
#SUFFIX ".pyd"
  )

target_link_libraries(${NAME}_ectomodule
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ecto
)
endmacro()
macro(link_ecto NAME)
target_link_libraries(${NAME}_ectomodule
 ${ARGN}
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

configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.in ${CMAKE_BINARY_DIR}/python_path.sh)
message(STATUS "To setup your python path for ecto you may source:
     ${CMAKE_BINARY_DIR}/python_path.sh")

set(ecto_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${LIBRARY_OUTPUT_PATH})
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)
message(STATUS "To use ecto in your own packages use
     find_library(ecto REQUIRED)
   and point cmake to the file: 
     ${CMAKE_BINARY_DIR}/ectoConfig.cmake")

set(BUILD_TEST_MODULES ON CACHE BOOL "Build test modules")

if(BUILD_TEST_MODULES)
  ectomodule(buster
    test/modules/buster.cpp
  )
  
find_package(OpenCV)
if(OpenCV_FOUND)
	ectomodule( imageproc 
		test/modules/imageproc.cpp
	)
	link_ecto(imageproc
		${OpenCV_LIBS})
	
endif(OpenCV_FOUND)

endif()

  
