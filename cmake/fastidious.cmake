option(USE_FASTIDIOUS_FLAGS
  "Use extra flags that help catch programming errors and prevent a build up of warnings"
  ON)

if(USE_FASTIDIOUS_FLAGS)
  if(UNIX)

    if(GCC_VERSION VERSION_LESS 4.5)
      set(CMAKE_CXX_FLAGS "-Werror -Wall -Wno-non-virtual-dtor")
    else()
      set(CMAKE_CXX_FLAGS "-Werror -Wall -Wl,--no-undefined")
    endif()

    if(${Boost_VERSION} VERSION_GREATER 1.40.0)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstrict-aliasing")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-strict-aliasing")
    endif()
    #apple is unix
    if(APPLE)
        #set(FASTIDIOUS_FLAGS "")
    endif()

  elseif(WIN32)
    set(CMAKE_CXX_FLAGS "/D _CRT_SECURE_NO_WARNINGS /D BOOST_HAS_STDINT_H /wd4099 /wd4101 /wd4996 /wd4251 /wd4305 /D _WIN32_WINNT=0x0501")
  endif()

  message(STATUS "USE_FASTIDIOUS_FLAGS is ON.  They are:  ${CMAKE_CXX_FLAGS}")

endif()

