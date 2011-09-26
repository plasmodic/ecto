find_package(Boost COMPONENTS
  python
  thread
  regex
  system
  REQUIRED
  )


try_run(BOOST_VERSION_RUN_RESULT BOOST_VERSION_COMPILE_RESULT
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/boost-version.c
  COMPILE_DEFINITIONS -I${Boost_INCLUDE_DIRS}
  COMPILE_OUTPUT_VARIABLE BOOST_VERSION_COMPILE
  RUN_OUTPUT_VARIABLE Boost_VERSION
  )

if(NOT BOOST_VERSION_COMPILE_RESULT)
  message(FATAL_ERROR "Couldn't compile boost version checking program: ${BOOST_VERSION_COMPILE}")
endif()

message(STATUS "Boost version ${Boost_VERSION}")

if (${Boost_VERSION} VERSION_EQUAL "1.40.0")
  include_directories(${ecto_SOURCE_DIR}/patches)
endif()

