find_package(Boost COMPONENTS
  python
  thread
  regex
  system
  serialization
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


macro(boost_feature_check checkname)
  try_compile(${checkname}
    ${CMAKE_BINARY_DIR}/${checkname}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/boost_checks.cpp
    COMPILE_DEFINITIONS -I${Boost_INCLUDE_DIRS} -D${checkname}=1
    OUTPUT_VARIABLE ${checkname}_OUTPUT
    )
  message("${checkname}: ${${checkname}}")
  if(${${checkname}_OUTPUT} MATCHES ".*ECTO_CHECK_TRY_COMPILE_ERROR.*")
    message(FATAL_ERROR "Internal error when checking for boost feature ${checkname}")
  endif()
endmacro()

add_definitions(${Boost_DEFINITIONS})

boost_feature_check(ECTO_EXCEPTION_SHARED_POINTERS_ARE_CONST)
boost_feature_check(ECTO_EXCEPTION_DIAGNOSTIC_IMPL_TAKES_CHARSTAR)
boost_feature_check(ECTO_EXCEPTION_RELEASE_RETURNS_VOID)

configure_file(${ecto_SOURCE_DIR}/cmake/boost-config.hpp.in ${ecto_BINARY_DIR}/include/ecto/boost-config.hpp)

