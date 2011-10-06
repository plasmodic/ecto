if(NOT ecto_SOURCE_DIR)
  cmake_minimum_required(VERSION 2.8)
  project(ecto-test)
  #this is for testing against an installed version of ecto
  #this is a full ecto based project
  find_package(ecto REQUIRED)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  enable_testing()
endif()
add_subdirectory(cells)
add_subdirectory(scripts)
add_subdirectory(cpp)
add_subdirectory(compile)
