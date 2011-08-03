#this helps with visual studio projects.
file(GLOB ecto_HEADERS
  ${CMAKE_SOURCE_DIR}/include/ecto/*.hpp
  ${CMAKE_SOURCE_DIR}/include/ecto/*/*.hpp
  )
source_group("Headers" FILES ${ecto_HEADERS})

add_library(ecto SHARED
  abi.cpp
  tendril.cpp
  tendrils.cpp
  cell.cpp
  plasm.cpp
  util.cpp
  log.cpp
  except.cpp
  needs_gil.cpp
  profile.cpp
  registry.cpp
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
    VERSION ${ECTO_VERSION}
    SOVERSION ${ECTO_SOVERSION}
    )
elseif(WIN32)
  set_target_properties(ecto PROPERTIES
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

