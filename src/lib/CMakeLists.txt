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
  registry.cpp
  serialization.cpp
  schedulers/invoke.cpp
  schedulers/singlethreaded.cpp
  schedulers/threadpool.cpp
  ${ecto_HEADERS}
  )

target_link_libraries(ecto
  ${ECTO_DEP_LIBS}
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

