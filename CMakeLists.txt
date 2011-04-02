#if(ROSBUILD)
#  include(rosbuild.cmake)
#  return()
#endif()
cmake_minimum_required(VERSION 2.8)
project(ecto)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

configure_file(python/ecto/__init__.py 
  ${LIBRARY_OUTPUT_PATH}/ecto/__init__.py
  @ONLY)

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
  
# Additional target properties
set_target_properties(ecto PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
        
ectomodule(ecto
  src/ecto_py/ecto.cpp
  src/ecto_py/plasm.cpp
  src/ecto_py/tendril.cpp
  src/ecto_py/module.cpp
  )

set_target_properties(ecto_ectomodule PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/share/ecto/python"
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
#disable samples
#add_subdirectory(samples)

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

INSTALL(FILES ${ecto_HEADERS}
  DESTINATION include/ecto
  COMPONENT ecto
  )

#install the main shared lib
install(TARGETS ecto
  LIBRARY DESTINATION lib
  COMPONENT ecto
  )

#create an ectoConfig.cmake for easy find_package(ecto)
set(ecto_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${CMAKE_INSTALL_PREFIX}/lib)
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake @ONLY)

#instal the ectoConfig.cmake
INSTALL(FILES ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake
  DESTINATION share/ecto
  COMPONENT ecto
  )

#install python stuff
set(ecto_PYTHON_INSTALL share/ecto/python)

#this is the python extension
install(TARGETS ecto_ectomodule
  LIBRARY DESTINATION ${ecto_PYTHON_INSTALL} COMPONENT ecto_python
  )

#python support
set(ecto_PYTHON_FILES
  python/ecto/__init__.py
  python/ecto/doc.py
  python/ecto/module_tools.py
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

add_subdirectory(samples)
