#cmake_minimum_required(VERSION 2.8)

project(ecto_samples)
set(BUILD_SAMPLES ON CACHE BOOL "Build sample modules")

if(BUILD_SAMPLES)

  #ectomodule(orb ORB/orb.cpp ORB/FASTHarris.cpp)
  #link_ecto(orb ${OpenCV_LIBS})

endif()
