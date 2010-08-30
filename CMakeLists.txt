cmake_minimum_required(VERSION 2.8)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(bang 
  src/main.cpp
  src/name_of.cpp)

target_link_libraries(bang rt)
		    