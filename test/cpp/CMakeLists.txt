find_package(GTest)

if(NOT GTEST_FOUND)
return()
endif()

add_executable(ecto-test
  main.cpp
  tendril.cpp
  spore.cpp
  exceptions.cpp
  graph.cpp
  )

target_link_libraries(ecto-test
  ecto_cpp
  ${GTEST_LIBRARIES}
  ${ECTO_DEP_LIBS}
  )

add_test(gtest_units ${EXECUTABLE_OUTPUT_PATH}/ecto-test)

