set(FASTIDIOUS_FLAGS "" CACHE STRING "Fastidious gcc flags")
option(USE_FASTIDIOUS_FLAGS 
  "Use extra flags that help catch programming errors and prevent a build up of warnings" ON)

if(USE_FASTIDIOUS_FLAGS)
  if(UNIX)
    set(FASTIDIOUS_FLAGS -Werror -Wall -Wno-non-virtual-dtor)

    if(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
      list(APPEND FASTIDIOUS_FLAGS -Wl,--Wno-undefined)
    endif()

  elseif(WIN32)

    set(FASTIDIOUS_FLAGS "/D _CRT_SECURE_NO_WARNINGS /D BOOST_HAS_STDINT_H /wd4099 /wd4101 /wd4996 /wd4251 /wd4305 /D _WIN32_WINNT=0x0501")

  elseif(APPLE)

    set(FASTIDIOUS_FLAGS "")

  endif()
endif()

add_definitions (${FASTIDIOUS_FLAGS})
