cmake_minimum_required(VERSION 2.8)
project(ecto)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

execute_process(COMMAND mkdir -p ${LIBRARY_OUTPUT_PATH}/ecto)

execute_process(COMMAND ln -s ${CMAKE_SOURCE_DIR}/python/ecto/__init__.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/lib/ecto
  )

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

#set flags so that we have awesome compile time errors on all warnings
#notice the -Wl,--no-undefined helps with making sure all symbols are found during
#link time.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wl,--no-undefined")
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

configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)

add_subdirectory(test)

#### install stuff #####

#install headers
set(ecto_HEADERS
  include/ecto/ecto.hpp
  include/ecto/module.hpp
  include/ecto/plasm.hpp
  include/ecto/tendril.hpp
  include/ecto/tendrils.hpp
  include/ecto/util.hpp
  )
set (ecto_PYTHON_HEADERS
  include/ecto/python/copy_suite.hpp
  include/ecto/python/raw_constructor.hpp
  include/ecto/python/repr.hpp
  include/ecto/python/std_map_indexing_suite.hpp
  include/ecto/python/std_vector_indexing_suite.hpp
  )

INSTALL(FILES ${ecto_HEADERS}
  DESTINATION include/ecto
  COMPONENT ecto
  )
INSTALL(FILES ${ecto_PYTHON_HEADERS}
  DESTINATION include/ecto/python
  COMPONENT ecto
  )
  
#create an ectoConfig.cmake for easy find_package(ecto)
set(ecto_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(ecto_PYTHON_INSTALL share/ecto/python)
set(ecto_PYTHONPATH ${CMAKE_INSTALL_PREFIX}/${ecto_PYTHON_INSTALL})
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake @ONLY)

#install the ectoConfig.cmake
INSTALL(FILES ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake
  DESTINATION share/ecto
  COMPONENT ecto
  )

#install python stuff


#python support
set(ecto_PYTHON_FILES
  python/ecto/__init__.py
  python/ecto/doc.py
  python/ecto/eyaml.py
  python/ecto/module.py
  python/ecto/xdot.py
)

install(FILES ${ecto_PYTHON_FILES}
  DESTINATION ${ecto_PYTHON_INSTALL}/ecto COMPONENT ecto_python
  )

configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.inst.in 
  ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
  )

install(FILES ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
        DESTINATION share/ecto COMPONENT ecto_python
  )
#add_subdirectory(samples)
add_subdirectory(doc)
add_subdirectory(src)
