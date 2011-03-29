#cmake_minimum_required(VERSION 2.8)

project(ecto_samples)
set(BUILD_SAMPLES ON CACHE BOOL "Build sample modules")

if(BUILD_SAMLES)

	#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
	#  svn co https://code.ros.org/svn/opencv/trunk
	#  and cmake that baby
	find_package(OpenCV)
	if(OpenCV_FOUND)
		ectomodule(orb ORB/orb.cpp ORB/FASTHarris.cpp)
		link_ecto(orb ${OpenCV_LIBS})
		ectomodule(calib lazer/calib.cpp)
		link_ecto(calib ${OpenCV_LIBS})
		ectomodule(lazer lazer/lazer.cpp)
		link_ecto(lazer ${OpenCV_LIBS})
		ectomodule(imageproc lazer/imageproc.cpp)
		link_ecto(imageproc ${OpenCV_LIBS})
	endif()

endif()