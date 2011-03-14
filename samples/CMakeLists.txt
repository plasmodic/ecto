cmake_minimum_required(VERSION 2.8)

project(ecto_sample)

find_package(ecto REQUIRED)

ectomodule(sample sample.cpp)


find_package(OpenCV)

if(OpenCV_FOUND)
ectomodule(orb ORB/orb.cpp ORB/FASTHarris.cpp)
link_ecto(orb ${OpenCV_LIBS})
endif()