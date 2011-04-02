# Try to find PCL library installation
#
# The follwoing variables are optionally searched for defaults
#  PCL_ROOT:             Base directory of PCL tree to use.
#  $ENV{PCL_ROOT}:       Base directory of PCL tree to use.
#  PCL_FIND_COMPONENTS : FIND_PACKAGE(PCL COMPONENTS ..) 
#    compatible interface. typically  common, io .. etc.
#
# The following are set after configuration is done: 
#  PCL_FOUND
#  PCL_INCLUDE_DIRS
#  PCL_LIBRARIES
#  PCL_LIBRARY_DIRS
#  PCL_VERSION
#
# In addition for each component these are set:
#  PCL_COMPONENT_INCLUDE_DIR
#  PCL_COMPONENT_LIBRARY
#  PCL_COMPONENT_DEFINITIONS if available
# 
# To use PCL from within you code
# find_package(PCL [VERSION] [REQUIRED] [COMPONENTS module1 module2 ...])
# if(PCL_FOUND)
#   include_directories(${PCL_INCLUDE_DIRS})
#   list(APPEND LIBS ${PCL_LBRARIES})
# endif(PCL_FOUND)
# Or if you want to link against a particular module
# target_link_libraries(my_fabulous_target ${PCL_XXX_LIBRARY}) where XXX is 
# module from COMPONENTS
#
# Tested with:
# -PCL 2.0
#
# www.pointclouds.org
# --------------------------------
include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
    set(pcl_all_components io common kdtree keypoints range_image registration sample_consensus segmentation features surface visualization )
    foreach(mod ${pcl_all_components})
      pkg_check_modules(PCL_MOD REQUIRED pcl_${mod})
      if(PCL_MOD_FOUND)
        list(APPEND PCL_INCLUDE_DIRS ${PCL_MOD_INCLUDE_DIRS})
        list(APPEND PCL_LIBRARIES ${PCL_MOD_LIBRARIES})
        list(APPEND PCL_LIBRARY_DIRS ${PCL_MOD_LIBRARY_DIRS})
        set(PCL_FOUND true)
      endif()
    endforeach()
    
    message(STATUS "Found : pcl_${mod} ->  ${PCL_MOD_INCLUDE_DIRS}")
    
else(PKG_CONFIG_FOUND)
    message(FATAL_ERROR "Could not find pkg-config to search for Eigen3.")
endif(PKG_CONFIG_FOUND)
