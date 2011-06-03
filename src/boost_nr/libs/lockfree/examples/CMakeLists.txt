set(examples
    fifo.cpp
    stack.cpp
)

link_libraries(boost_thread)

foreach(ex ${examples})
  string(REPLACE .cpp "" example_name ${ex} )
  add_executable(${example_name} ${ex})
endforeach(ex)
