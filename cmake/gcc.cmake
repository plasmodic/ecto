execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
                        OUTPUT_VARIABLE GCC_VERSION
                        OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "GCC version:               ${GCC_VERSION}")

