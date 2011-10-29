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
macro(setup_virtualenv PROJECT)
  option(INSTALL_TO_VIRTUALENV "Install by default to a virtual environment" OFF)
  option(WORK_IN_VIRTUALENV "Install by default to a virtual environment" OFF)
  if(WORK_IN_VIRTUALENV)
    if("$ENV{VIRTUAL_ENV}" STREQUAL "")
      message(WARNING "No active virtualenv!")
    else()
      set(VIRTUALENV_DIR $ENV{VIRTUAL_ENV} CACHE PATH "VIRTUALENV path" FORCE)
      set(INSTALL_TO_VIRTUALENV OFF CACHE BOOL "" FORCE) #MUTUALLY EXCLUSIVE
      set(CMAKE_INSTALL_PREFIX ${VIRTUALENV_DIR} CACHE PATH "" FORCE)
      set(CMAKE_INSTALL_RPATH  ${VIRTUALENV_DIR}/lib CACHE PATH "" FORCE)
      message(STATUS "virtualenv path:       ${VIRTUALENV_DIR}")
    endif()
  endif()
  if(INSTALL_TO_VIRTUALENV)
    find_program(VIRTUALENV_EXECUTABLE
      NAMES virtualenv
      DOC "The virtualenv executable."
    )
    mark_as_advanced(VIRTUALENV_EXECUTABLE)

    set(VIRTUALENV_DIR  "/opt/ecto/${PROJECT}/${${PROJECT}_GITTAG_SHORT}" CACHE PATH "Virtual environment")

    option(VIRTUALENV_ALL "Add virtualenv to all" off)
    mark_as_advanced(VIRTUALENV_ALL)
    if(VIRTUALENV_ALL)
        set(VIRTUALENV_ALL_ ALL)
    else()
        set(VIRTUALENV_ALL_)
    endif()
    
    add_custom_target(bootstrap ${VIRTUALENV_ALL_})
    add_custom_command(TARGET bootstrap
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/ecto/kitchen/cmake/bootstrap_gen.py
    )
    add_custom_target(virtualenv ${VIRTUALENV_ALL_})
    add_custom_command(TARGET virtualenv
      COMMAND ${PYTHON_EXECUTABLE} bootstrap.py ${VIRTUALENV_DIR}
    )
    
    add_dependencies(virtualenv bootstrap)
    set(CMAKE_INSTALL_PREFIX ${VIRTUALENV_DIR} CACHE PATH "" FORCE)
    set(CMAKE_INSTALL_RPATH  ${VIRTUALENV_DIR}/lib CACHE PATH "" FORCE)
    message(STATUS "virtualenv path:       ${VIRTUALENV_DIR}")
  endif()
endmacro()

