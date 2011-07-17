add_library(ecto SHARED
  abi.cpp
  tendril.cpp
  tendrils.cpp
  cell.cpp
  plasm.cpp
  util.cpp
  log.cpp
  except.cpp
  profile.cpp
  scheduler/invoke.cpp
  scheduler/singlethreaded.cpp
  scheduler/threadpool.cpp
  )
  
target_link_libraries(ecto
  ${ECTO_DEP_LIBS}
  )
  
set_target_properties(ecto PROPERTIES
            VERSION ${ECTO_VERSION}
            SOVERSION ${ECTO_SOVERSION}
            )


#install the main shared lib
install(TARGETS ecto
  DESTINATION lib
  COMPONENT main
  )

