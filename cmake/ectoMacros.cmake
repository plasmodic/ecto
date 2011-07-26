macro(ectomodule NAME)
  if(WIN32)
    link_directories(${Boost_LIBRARY_DIRS})
    set(ECTO_MODULE_DEP_LIBS 
      ${PYTHON_LIBRARIES}
      ${Boost_PYTHON_LIBRARY}
      )
  else()
    set(ECTO_MODULE_DEP_LIBS
      ${Boost_LIBRARIES}
      ${PYTHON_LIBRARIES}
      )
  endif()
  #these are required includes for every ecto module
  include_directories(
    ${ecto_INCLUDE_DIRS} 
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
    )
  
  add_library(${NAME}_ectomodule SHARED
    ${ARGN}
    )
  if(UNIX)
    set_target_properties(${NAME}_ectomodule
      PROPERTIES
      OUTPUT_NAME ${NAME}
      PREFIX ""
      )
  elseif(WIN32)
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import distutils.sysconfig; print distutils.sysconfig.get_config_var('SO')"
      RESULT_VARIABLE PYTHON_PY_PROCESS
      OUTPUT_VARIABLE PY_SUFFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    set_target_properties(${NAME}_ectomodule
      PROPERTIES
      OUTPUT_NAME ${NAME}
      PREFIX ""
      SUFFIX ${PY_SUFFIX}
      )
    message(STATUS "Using PY_SUFFIX = ${PY_SUFFIX}")
  endif()
  
  target_link_libraries(${NAME}_ectomodule
    ${ECTO_MODULE_DEP_LIBS}
    ${ecto_LIBRARIES}
    )
endmacro()

# ==============================================================================

macro(ectorosmodule NAME)
  if(WIN32)
    link_directories(${Boost_LIBRARY_DIRS})
    set(ECTO_MODULE_DEP_LIBS 
      ${PYTHON_LIBRARIES}
      ${Boost_PYTHON_LIBRARY}
      )
  else()
    set(ECTO_MODULE_DEP_LIBS
      ${Boost_LIBRARIES}
      ${PYTHON_LIBRARIES}
      )
  endif()
  include_directories(
    ${ecto_INCLUDE_DIRS} 
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
    )
  
  rosbuild_add_library(${NAME}_ectomodule 
    ${ARGN}
    )
  
  set_target_properties(${NAME}_ectomodule
    PROPERTIES
    OUTPUT_NAME ${NAME}
    PREFIX ""
    )
  
  target_link_libraries(${NAME}_ectomodule
    ${ECTO_MODULE_DEP_LIBS}
    ${ecto_LIBRARIES}
    )
endmacro()

# ==============================================================================

macro(link_ecto NAME)
  target_link_libraries(${NAME}_ectomodule
    ${ARGN}
    )
endmacro()

# ==============================================================================
#this is where usermodules may be installed to
macro( set_ecto_install_package_name package_name)
  set(ecto_module_PYTHON_INSTALL ${PYTHON_PACKAGES_PATH}/${package_name})
endmacro()

# ==============================================================================
macro( install_ecto_module name )
  #this is the python extension
  #message("installing ${name}_ectomodule to ${ecto_module_PYTHON_INSTALL}")
  #install(TARGETS ${name}_ectomodule
  #  LIBRARY DESTINATION ${ecto_module_PYTHON_INSTALL}
  #  COMPONENT main
  #  )
endmacro()
# ==============================================================================
