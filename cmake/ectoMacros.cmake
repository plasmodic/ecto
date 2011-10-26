# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
option(ECTO_LOG_STATS "Generate logs containing fine-grained per-cell execution timing information.  You probably don't want this."
  OFF)
mark_as_advanced(ECTO_LOG_STATS)

if(ECTO_LOG_STATS)
  add_definitions(-DECTO_LOG_STATS=1)
endif()


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
      COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
      LINK_FLAGS -shared-libgcc
      PREFIX ""
      )
  elseif(WIN32)
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import distutils.sysconfig; print distutils.sysconfig.get_config_var('SO')"
      RESULT_VARIABLE PYTHON_PY_PROCESS
      OUTPUT_VARIABLE PY_SUFFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    set_target_properties(${NAME}_ectomodule
      PROPERTIES
      COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
      OUTPUT_NAME ${NAME}
      PREFIX ""
      SUFFIX ${PY_SUFFIX}
      )
    message(STATUS "Using PY_SUFFIX = ${PY_SUFFIX}")
  endif()
  if(APPLE)
    set_target_properties(${NAME}_ectomodule
      PROPERTIES
      SUFFIX ".so"
      )
  endif()

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
  install(TARGETS ${name}_ectomodule
    DESTINATION ${ecto_module_PYTHON_INSTALL}
    COMPONENT main
  )
endmacro()
# ==============================================================================

# ============== Python Path ===================================================

#ecto_python_env_gen( [path1 [ path2 [ path3 [...] ] ] ])
#this will generate a python_path.sh script in the ${PROJECT_BINARY_DIR}
#where any argument supplied is consired to be a python module directory
#and will be preprended to the python path.  Also, the path where ecto's python
#binding are located will appear after any user supplied lists.
#typically this call will look like:
# ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
#
macro( ecto_python_env_gen )
    set(ecto_PYTHONPATH_ ${ecto_PYTHONPATH} )
    set(ecto_user_PYTHONPATH )
    #put user path first
    list(APPEND ecto_user_PYTHONPATH  ${ARGN})
    list(APPEND ecto_user_PYTHONPATH  ${ecto_PYTHONPATH})
    #transform the cmake list to a sh path list
    string(REPLACE ";" ":"
        ecto_user_PYTHONPATH
        "${ecto_user_PYTHONPATH}"
    )
    configure_file(${ECTO_CONFIG_PATH}/python_path.sh.user.in 
      ${PROJECT_BINARY_DIR}/python_path.sh
      )
    file(RELATIVE_PATH nice_path_ ${CMAKE_BINARY_DIR} ${PROJECT_BINARY_DIR}/python_path.sh)
    if (NOT ecto_kitchen_SOURCE_DIR)
      message(STATUS "To setup your python path for *${PROJECT_NAME}* you may source: ${nice_path_}")
    endif()
endmacro()


