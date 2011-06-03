set(tests
    fifo_test.cpp
    freelist_test.cpp
    ringbuffer_test.cpp
    stack_test.cpp
    tagged_ptr_test.cpp
)

link_libraries(boost_thread)

# build benchmarks
foreach(test ${tests})
  string(REPLACE .cpp "" test_name ${test} )
  add_executable(${test_name} ${test})
  add_test(${test_name}_run ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endforeach(test)
