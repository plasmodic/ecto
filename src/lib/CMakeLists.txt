#this helps with visual studio projects.
file(GLOB ecto_HEADERS
  ${CMAKE_SOURCE_DIR}/include/ecto/*.hpp
  ${CMAKE_SOURCE_DIR}/include/ecto/*/*.hpp
  )
source_group("Headers" FILES ${ecto_HEADERS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(ecto SHARED
  abi.cpp
  cell.cpp
  edge.cpp
  tendril.cpp
  tendrils.cpp
  plasm.cpp
  plasm/impl.cpp
  util.cpp
  log.cpp
  except.cpp
  needs_gil.cpp
  profile.cpp
  python.cpp
  registry.cpp
  rethrow.cpp
  serialization.cpp
  scheduler.cpp
  schedulers/invoke.cpp
  schedulers/singlethreaded.cpp
  schedulers/multithreaded.cpp
  # schedulers/threadpool.cpp
  ${ecto_HEADERS}
  )

target_link_libraries(ecto
  ${ECTO_DEP_LIBS}
  )

set_source_files_properties(log.cpp
  PROPERTIES
  COMPILE_FLAGS -DSOURCE_DIR=\\"${ecto_SOURCE_DIR}\\"
  )

if(UNIX)
  set_target_properties(ecto PROPERTIES
    COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
    VERSION ${ECTO_VERSION}
    SOVERSION ${ECTO_SOVERSION}
    )
elseif(WIN32)
  set_target_properties(ecto PROPERTIES
    COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
    VERSION ${ECTO_VERSION}
    SOVERSION ${ECTO_SOVERSION}
    OUTPUT_NAME ecto_cpp
    )
endif()

#install the main shared lib
install(TARGETS ecto
  DESTINATION lib
  COMPONENT main
  )

