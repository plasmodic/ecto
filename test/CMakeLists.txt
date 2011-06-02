project(ecto_tests)

ectomodule(buster
    modules/buster.cpp
)

add_executable(push_ups
	benchmarks/main.cpp
)

target_link_libraries(push_ups
	${Boost_LIBRARIES}
	${PYTHON_LIBRARIES}
	ecto
)
