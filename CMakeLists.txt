cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS -Wall)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

macro(myexec NAME)

  add_executable(${NAME}
    ${ARGN})
  target_link_libraries(${NAME} rt boost_date_time boost_system)
  
endmacro()

myexec(publisher 
  src/publisher.cpp
  src/name_of.cpp
  )
		    
myexec(subscriber 
  src/subscriber.cpp
  src/name_of.cpp
  )
 
myexec(remover
  src/remove.cpp)

myexec(asio_srv
  src/asio_srv.cpp
  src/util.cpp)

myexec(asio_client
  src/asio_client.cpp)



		    

myexec(poolio
  src/poolio.cpp
  )