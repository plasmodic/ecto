find_package(GTest)

if(NOT GTEST_FOUND)
return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(ecto-test
  main.cpp
  tendril.cpp
  tendrils.cpp
  spore.cpp
  exceptions.cpp
  graph.cpp
  )

target_link_libraries(ecto-test
  ecto_cpp
  ${GTEST_LIBRARIES}
  ${ECTO_DEP_LIBS}
  )

add_test(ecto_gtest_units ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ecto-test)
