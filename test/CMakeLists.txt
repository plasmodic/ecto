project(ecto_tests)

set(BUILD_TEST_MODULES ON CACHE BOOL "Build test modules")

if(BUILD_TEST_MODULES)
  ectomodule(buster
    modules/buster.cpp
    )

  ectomodule(push_ups
    benchmarks/push_ups.cpp
  )
    

add_executable(push_ups
	benchmarks/main.cpp
)

target_link_libraries(push_ups
	${Boost_LIBRARIES}
	${PYTHON_LIBRARIES}
	ecto
)
  

endif()