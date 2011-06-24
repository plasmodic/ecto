add_library(ecto SHARED
  tendril.cpp
  tendrils.cpp
  module.cpp
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
  
# Additional target properties
set_target_properties(ecto PROPERTIES
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
  )

set_target_properties(ecto PROPERTIES
            VERSION ${ECTO_VERSION}
            SOVERSION ${ECTO_SOVERSION}
            )

#install the main shared lib
install(TARGETS ecto
  LIBRARY DESTINATION lib
  COMPONENT ecto
  )

