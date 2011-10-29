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
find_package(Threads)

find_package(PythonLibs REQUIRED)

include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  )

if(WIN32)
  link_directories(${Boost_LIBRARY_DIRS})
  set(ECTO_DEP_LIBS
    ${PYTHON_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_PYTHON_LIBRARY}
    CACHE INTERNAL "Libraries dependencies" FORCE
  )
else()
  set(ECTO_DEP_LIBS
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    CACHE INTERNAL "Libraries dependencies" FORCE
  )
endif()

#detect the python version and install directories
find_package(PythonInterp REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE} --version
          ERROR_VARIABLE PYTHON_VERSION_FULL
          OUTPUT_STRIP_TRAILING_WHITESPACE)

string(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")
mark_as_advanced(PYVERSIONS_EXE)
if(UNIX)
    if(APPLE)
        set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages CACHE INTERNAL "Where to install the python packages.")
    else() #debian based assumed, install to the dist-packages.
        set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/dist-packages CACHE INTERNAL "Where to install the python packages.")
    endif()
elseif(WIN32)
    get_filename_component(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    set(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages" CACHE INTERNAL "Where to install the python packages.")
endif()

set(ecto_module_PYTHON_INSTALL_base ${PYTHON_PACKAGES_PATH} CACHE INTERNAL "The base path where ecto modules will be installed." )
set(ecto_module_PYTHON_INSTALL ${ecto_module_PYTHON_INSTALL_base})
#this is where usermodules may be installed to
macro(set_ecto_install_package_name package_name)
  set(ecto_module_PYTHON_INSTALL ${ecto_module_PYTHON_INSTALL_base}/${package_name})
endmacro()

set(ecto_PYTHON_INSTALL ${PYTHON_PACKAGES_PATH})

