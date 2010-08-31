cmake_minimum_required(VERSION 2.8)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(publisher 
  src/publisher.cpp
  src/name_of.cpp
  )
target_link_libraries(publisher rt boost_date_time boost_system)
		    
add_executable(subscriber 
  src/subscriber.cpp
  src/name_of.cpp
  )
target_link_libraries(subscriber rt boost_date_time boost_system)
		    

