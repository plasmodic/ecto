include_directories(lib)
add_subdirectory(lib)
add_subdirectory(pybindings)

macro(asio_executable name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name}
    ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endmacro()

asio_executable(asio_sketch)
asio_executable(asio_threadpool)
asio_executable(graph_sketch)

