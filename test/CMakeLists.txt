project(ecto_tests)

ectomodule(ecto_test
    modules/ecto_test.cpp
)


add_executable(push_ups
  benchmarks/main.cpp
)

target_link_libraries(push_ups
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ecto
)

add_subdirectory(scripts)