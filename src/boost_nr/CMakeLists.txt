cmake_minimum_required (VERSION 2.6)
project (boost-lockfree)

enable_testing()

include_directories(.)

add_subdirectory (libs/lockfree/doc)
add_subdirectory (libs/lockfree/examples)
add_subdirectory (libs/lockfree/test)
