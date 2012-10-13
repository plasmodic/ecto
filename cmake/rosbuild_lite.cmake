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

function(check_unused_arguments ARG_NAME ARG_UNUSED)
  if(ARG_UNUSED)
    message(WARNING "${ARG_NAME} called with unused arguments: ${ARG_UNUSED}")
  endif()
endfunction()

option(ROS_CONFIGURE_VERBOSE OFF)

unset(ROS_FUERTE_FOUND)
unset(ROS_GROOVY_FOUND)
unset(ROS_GROOVY_OR_ABOVE_FOUND)

    set(BUNCH_OF_VARS "$ENV{ROS_ROOT}, ${CMAKE_PREFIX_PATH}, $ENV{ROS_PACKAGE_PATH}, ${CMAKE_INSTALL_PREFIX}, ${catkin_INSTALL_PREFIX}, ${catkin_EXTRAS_DIR}")
    string(REGEX MATCH "fuerte" ROS_FUERTE_FOUND ${BUNCH_OF_VARS})
    if (ROS_FUERTE_FOUND)
        set(ROS_FUERTE_FOUND TRUE)
    else()
        set(ROS_GROOVY_FOUND TRUE)
        set(ROS_GROOVY_OR_ABOVE_FOUND TRUE)
    endif()


#attempts to set ENV variables so that ROS commands will work.
#This appears to work well on linux, but may be questionable on
#other platforms.
macro (_set_ros_env)
  set(ORIG_ROS_ROOT $ENV{ROS_ROOT})
  set(ORIG_ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
  set(ORIG_PATH $ENV{PATH})
  set(ORIG_PYTHONPATH $ENV{PYTHONPATH})
  set(ENV{ROS_ROOT} ${ROS_ROOT})
  set(ENV{ROS_PACKAGE_PATH} ${ROS_PACKAGE_PATH})
  set(ENV{PATH} "${ROS_ROOT}/bin:$ENV{PATH}")
  set(ENV{PYTHONPATH} "${ROS_ROOT}/core/roslib/src:$ENV{PYTHONPATH}")
endmacro()

#unset environment
macro (_unset_ros_env)
  set(ENV{ROS_ROOT} ${ORIG_ROS_ROOT})
  set(ENV{ROS_PACKAGE_PATH} ${ORIG_ROS_PACKAGE_PATH})
  set(ENV{PATH} "${ORIG_PATH}")
  set(ENV{PYTHONPATH} "${ORIG_PYTHONPATH}")
endmacro()

# find the (unstable) pcl16 package. uses rosbuild, even on fuerte,
# since perception_pcl_fuerte_unstable isn't yet catkinized
macro(find_pcl16_package)
  unset(PCL16_LIBRARIES)
  find_ros_package(std_msgs)
  find_ros_package(pcl16)
  set(PCL16_FOUND TRUE)
  set(PCL16_LIBRARIES ${pcl16_LIBRARIES})
  set(PCL16_INCLUDE_DIRS ${pcl16_INCLUDE_DIRS})
  message(STATUS "+   ${pcl16_libraries} libraries, ${pcl16_include_dirs} include directories")
endmacro()
