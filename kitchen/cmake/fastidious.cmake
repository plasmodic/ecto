set(FASTIDIOUS_FLAGS "" CACHE STRING "Fastidious gcc flags")
option(USE_FASTIDIOUS_FLAGS 
  "Use extra flags that help catch programming errors and prevent a build up of warnings" ON)

if(USE_FASTIDIOUS_FLAGS)
  if(UNIX)
    #be sure to keep the flags as a string and not as cmake list.
    #These are used in the ectomodule macro like, so that not all targets are
    #forced to use them.
    #
    #set_target_properties(mytarget
    #  PROPERTIES
    #  COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
    #  )
    set(FASTIDIOUS_FLAGS "-Werror -Wall")
    if(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
      set(FASTIDIOUS_FLAGS "${FASTIDIOUS_FLAGS} -Wl,--Wno-undefined")
    else()
      set(FASTIDIOUS_FLAGS "${FASTIDIOUS_FLAGS} -Wno-non-virtual-dtor")
    endif()
    
    #apple is unix
    if(APPLE)
        #set(FASTIDIOUS_FLAGS "")
    endif()
    
  elseif(WIN32)
    set(FASTIDIOUS_FLAGS "/D _CRT_SECURE_NO_WARNINGS /D BOOST_HAS_STDINT_H /wd4099 /wd4101 /wd4996 /wd4251 /wd4305 /D _WIN32_WINNT=0x0501")
  endif()
endif()
