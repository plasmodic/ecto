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
option(USE_FASTIDIOUS_FLAGS
  "Use extra flags that help catch programming errors and prevent a build up of warnings"
  ON)

if(USE_FASTIDIOUS_FLAGS)
  if(UNIX)

    if(GCC_VERSION VERSION_LESS 4.5)
      set(CMAKE_CXX_FLAGS "-Werror -Wall -Wno-non-virtual-dtor")
    else()
      set(CMAKE_CXX_FLAGS "-Werror -Wall -Wl,--no-undefined")
    endif()

    if(${Boost_VERSION} VERSION_GREATER 1.40.0)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstrict-aliasing")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-strict-aliasing")
    endif()
    #apple is unix
    if(APPLE)
        #set(FASTIDIOUS_FLAGS "")
    endif()

  elseif(WIN32)
    set(CMAKE_CXX_FLAGS "/D _CRT_SECURE_NO_WARNINGS /D BOOST_HAS_STDINT_H /wd4099 /wd4101 /wd4996 /wd4251 /wd4305 /D _WIN32_WINNT=0x0501")
  endif()

  message(STATUS "USE_FASTIDIOUS_FLAGS is ON.  They are:  ${CMAKE_CXX_FLAGS}")

endif()


