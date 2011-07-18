cmake_minimum_required(VERSION 2.8)

project(ecto_samples)
    
#find_package(ecto REQUIRED)
    
ectomodule(hello_ecto
    hello_ecto.cpp
)

  #these are required includes for every ecto module
include_directories(
${PYTHON_INCLUDE_PATH}
${Boost_INCLUDE_DIRS}
)

add_library(hellopy SHARED
hellopy.cpp
)
set_target_properties(hellopy
  PROPERTIES
  OUTPUT_NAME hellopy
  PREFIX ""
  SUFFIX .pyd
  )
link_directories(${PYTHON_LIBRARY_DIRS})
target_link_libraries(hellopy
${PYTHON_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
${Boost_PYTHON_LIBRARY}
)

#optionally link against other libs
#ecto_link(hello_ecto
#  ${MY_EXTRA_LIBS}
#)

#set_ecto_install_package_name(ecto_samples)

#generates an install target for your module (puts it in the python dist folder)
install_ecto_module(hello_ecto)

#install the main shared lib
install(PROGRAMS ecto_doc.py
  DESTINATION bin
  COMPONENT ecto_python
  )

add_subdirectory(tutorial)
