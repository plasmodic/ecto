#this helps with visual studio projects.
file(GLOB ecto_HEADERS
	${CMAKE_SOURCE_DIR}/include/ecto/*.hpp
	${CMAKE_SOURCE_DIR}/include/ecto/*/*.hpp
	)
source_group("Headers" FILES ${ecto_HEADERS})

add_library(ecto_cpp SHARED
  abi.cpp
  tendril.cpp
  tendrils.cpp
  cell.cpp
  plasm.cpp
  util.cpp
  log.cpp
  except.cpp
  profile.cpp
  registry.cpp
  schedulers/invoke.cpp
  schedulers/singlethreaded.cpp
  schedulers/threadpool.cpp
  ${ecto_HEADERS}
  )
  
target_link_libraries(ecto_cpp
  ${ECTO_DEP_LIBS}
  )

if(UNIX)
set_target_properties(ecto_cpp PROPERTIES
            VERSION ${ECTO_VERSION}
            SOVERSION ${ECTO_SOVERSION}
            )
elseif(WIN32)
#set_target_properties(ecto_cpp PROPERTIES
#            VERSION ${ECTO_VERSION}
#            SOVERSION ${ECTO_SOVERSION}
#            )
endif()

#install the main shared lib
install(TARGETS ecto_cpp
  DESTINATION lib
  COMPONENT main
  )

