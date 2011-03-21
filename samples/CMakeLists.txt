#cmake_minimum_required(VERSION 2.8)

project(ecto_sample)

#find_package(ecto REQUIRED)

find_package(OpenCV)
if(OpenCV_FOUND)
ectomodule(orb ORB/orb.cpp ORB/FASTHarris.cpp)
link_ecto(orb ${OpenCV_LIBS})
ectomodule(calib lazer/calib.cpp)
link_ecto(calib ${OpenCV_LIBS})
ectomodule(lazer lazer/lazer.cpp)
link_ecto(lazer ${OpenCV_LIBS})

endif()
