cmake_minimum_required(VERSION 2.8)

project(ecto)

set(CMAKE_CXX_FLAGS -Wall)


find_package(PythonLibs)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PYTHON_INCLUDE_PATH}
  )

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


#myexec(poolio src/poolio.cpp)

#myexec(rcc src/rcc.cpp)
#myexec(vec src/vec.cpp)
#myexec(doc_cont src/doc_cont.cpp)



add_library(ecto-pybindings SHARED
  src/name_of.cpp
  src/bindings.cpp
  )

set_target_properties(ecto-pybindings
  PROPERTIES
  OUTPUT_NAME ecto
  PREFIX ""
  )

target_link_libraries(ecto-pybindings
  boost_python
  ${PYTHON_LIBRARIES})

  