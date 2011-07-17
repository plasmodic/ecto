if(WIN32)
	link_directories(${Boost_LIBRARY_DIRS})
	set(ECTO_MODULE_DEP_LIBS 
		${PYTHON_LIBRARIES}
		CACHE STRING "Ecto user module libraries dependencies" FORCE
	)
else()
	set(ECTO_MODULE_DEP_LIBS
	  ${Boost_LIBRARIES}
	  ${PYTHON_LIBRARIES}
	  CACHE STRING "Ecto user module libraries dependencies" FORCE
	)
endif()

macro(ectomodule NAME)
    #these are required includes for every ecto module
    include_directories(
      ${ecto_INCLUDE_DIRS} 
      ${PYTHON_INCLUDE_PATH}
      ${Boost_INCLUDE_DIRS}
    )
    
    add_library(${NAME}_ectomodule SHARED
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

macro(ectorosmodule NAME)
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
      ${Boost_LIBRARIES}
      ${PYTHON_LIBRARIES}
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
    install(TARGETS ${name}_ectomodule
      DESTINATION ${ecto_module_PYTHON_INSTALL}
	  COMPONENT main
      )
endmacro()
# ==============================================================================