find_program(ROSPACK_EXECUTABLE rospack DOC "the rospack executable.")

macro (rospack VAR COMMAND PACKAGE)
  set(cachevar ROSPACK_${PACKAGE}_${COMMAND})
  set(${cachevar} "not-run-yet-NOTFOUND" CACHE INTERNAL "")
  if(NOT ${cachevar})
    execute_process(COMMAND ${ROSPACK_EXECUTABLE} ${COMMAND} ${PACKAGE}
      OUTPUT_VARIABLE ROSPACK_OUT
      ERROR_VARIABLE rospack_error
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
      )

    if(rospack_error)
        message(STATUS "***")
        message(STATUS "*** rospack ${COMMAND} ${PACKAGE} failed: ${rospack_error}")
        message(STATUS "***")
        set(${cachevar} "ROSPACK_${PACKAGE}_${COMMAND}-NOTFOUND" 
          CACHE INTERNAL "rospack output for rospack ${PACKAGE} ${COMMAND}")
    else()
        separate_arguments(ROSPACK_SEPARATED UNIX_COMMAND ${ROSPACK_OUT})
        set(${cachevar} ${ROSPACK_SEPARATED} CACHE INTERNAL "value")
        set(${VAR} ${ROSPACK_SEPARATED} CACHE INTERNAL "" FORCE)
        # message("${VAR} == ${${VAR}}")
    endif()
  else()
    set(${VAR} "${${cachevar}}" CACHE INTERNAL "" FORCE)
  endif()
endmacro()

macro (find_ros_package PACKAGE)
  rospack(${PACKAGE}_DIR find ${PACKAGE})

  if(NOT ${PACKAGE}_DIR)
    message(STATUS "Could not find package ${PACKAGE} via rosmake")
  elseif(NOT ${PACKAGE}_FOUND)
    message(STATUS "Finding ROS package ${PACKAGE} via rospack and ROS environment variables...")
    rospack(${PACKAGE}_INCLUDE_DIRS cflags-only-I ${PACKAGE})
    include_directories(${${PACKAGE}_INCLUDE_DIRS})
    rospack(${PACKAGE}_DEFINITIONS cflags-only-other ${PACKAGE})
    foreach(DEF ${${PACKAGE}_DEFINITIONS})
      add_definitions(" ${DEF}")
    endforeach()
    rospack(libdirs libs-only-L ${PACKAGE})
    rospack(libnames libs-only-l ${PACKAGE})
    set(${PACKAGE}_LIBRARIES "" CACHE STRING "libs to link against when depending on ${PACKAGE}" FORCE)

    # message(STATUS "Finding libraries ${libnames}")
    # message(STATUS "in directories    ${libdirs}")
    #foreach(p ${libdirs})
    #  message("bing=${p}")
    #endforeach()
    foreach(libname ${libnames})
      #message("finding ${libname}")
      find_library(${libname}_LIBRARY
        NAMES ${libname}
        PATHS ${libdirs}
        NO_DEFAULT_PATH
        )
      find_library(${libname}_LIBRARY ${libname})
      # message("${libname}_LIBRARY ${${libname}_LIBRARY}")
      if (NOT ${libname}_LIBRARY)
        message(FATAL_ERROR "uh oh ${PACKAGE} ${libname} found us ${thelib}")
      endif()
      list(APPEND ${PACKAGE}_LIBRARIES ${${libname}_LIBRARY})
      # link_directories(${${PACKAGE}_LIBRARY_DIRS})
    endforeach()
    # message("${PACKAGE}_LIBRARIES ${${PACKAGE}_LIBRARIES}")
  endif()


  list(LENGTH ${PACKAGE}_LIBRARIES nlibs)
  list(LENGTH ${PACKAGE}_INCLUDE_DIRS nincludes)
  list(LENGTH ${PACKAGE}_DEFINITIONS ndefs)

  if (${PACKAGE}_DIR)
    message(STATUS "+ ${PACKAGE} at ${${PACKAGE}_DIR}")
    message(STATUS "+   ${nlibs} libraries, ${nincludes} include directories, ${ndefs} compile definitions")
    set(${PACKAGE}_FOUND TRUE CACHE INTERNAL "" FORCE)
  else()
    message(STATUS "+ ${PACKAGE}: NOT FOUND")
    set(${PACKAGE}_FOUND FALSE CACHE INTERNAL "" FORCE)
  endif()

endmacro()
