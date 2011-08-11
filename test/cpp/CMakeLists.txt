find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${ecto_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
)

add_executable(ecto-test
  main.cpp
  tendril.cpp
  tendrils.cpp
  spore.cpp
  exceptions.cpp
  graph.cpp
  serial.cpp
  threadpool.cpp
  clone.cpp
  static.cpp
  )

target_link_libraries(ecto-test
  ecto
  ${GTEST_LIBRARIES}
  ${ECTO_DEP_LIBS}
  # this is so that we can create cells that are in the test lib w/o 
  # python importing them
  ecto_test_ectomodule
  ecto_ectomodule
  )

add_test(ecto_gtest_units ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ecto-test)

add_executable(plasm_loader
  plasm_loader.cpp
  )


target_link_libraries(plasm_loader
  ecto
  ${ECTO_DEP_LIBS}
  # this is so that we can create cells that are in the test lib w/o 
  # python importing them
  ecto_test_ectomodule
  ecto_ectomodule
)
  
add_test(plasm_loader
    ${CMAKE_BINARY_DIR}/bin/plasm_loader
    ${CMAKE_CURRENT_SOURCE_DIR}/python_graph.ecto
    8
)