cmake_minimum_required(VERSION 2.8)
project(ecto)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
configure_file(python/ecto/__init__.py 
  ${CMAKE_BINARY_DIR}/lib/ecto/__init__.py
  @ONLY)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wl,--no-undefined")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBOOST_DISABLE_ASSERTS")

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
  src/tendril.cpp
  src/module.cpp
  src/plasm.cpp
  src/util.cpp
  )
target_link_libraries(ecto
	${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
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

set(ecto_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${LIBRARY_OUTPUT_PATH})

configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)

set(BUILD_TEST_MODULES ON CACHE BOOL "Build test modules")

if(BUILD_TEST_MODULES)
  ectomodule(buster
    test/modules/buster.cpp
    )
  #use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
  #  svn co https://code.ros.org/svn/opencv/trunk
  #or the release https://code.ros.org/svn/opencv/branches/2.2/opencv
  #and cmake that baby
  find_package(OpenCV)

  if(OpenCV_FOUND)
    #message("===${OpenCV_INCLUDE_DIRS}")
    #include_directories(${OpenCV_INCLUDE_DIRS})
    ectomodule( imageproc 
      test/modules/imageproc.cpp
      )
    link_ecto(imageproc
      ${OpenCV_LIBS})
  endif(OpenCV_FOUND)
  add_subdirectory(samples)
  ectomodule(push_ups
    test/benchmarks/push_ups.cpp
    )
    
    add_executable(push_ups
    	test/benchmarks/main.cpp
    )
    target_link_libraries(push_ups
    	${Boost_LIBRARIES}
    	${PYTHON_LIBRARIES}
    	ecto
    )
  

endif()


