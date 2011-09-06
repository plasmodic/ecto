macro(setup_virtualenv PROJECT)
  option(INSTALL_TO_VIRTUALENV "Install by default to a virtual environment" ON)
  option(WORK_IN_VIRTUALENV "Install by default to a virtual environment" OFF)
  if(WORK_IN_VIRTUALENV)
    if("$ENV{VIRTUAL_ENV}" STREQUAL "")
      message(WARNING "No active virtualenv!")
    else()
      set(VIRTUALENV_DIR $ENV{VIRTUAL_ENV} CACHE PATH "VIRTUALENV path" FORCE)
      set(INSTALL_TO_VIRTUALENV OFF CACHE BOOL "" FORCE) #MUTUALLY EXCLUSIVE
      set(CMAKE_INSTALL_PREFIX ${VIRTUALENV_DIR} CACHE PATH "" FORCE)
      set(CMAKE_INSTALL_RPATH  ${VIRTUALENV_DIR}/lib CACHE PATH "" FORCE)
      message(STATUS "virtualenv path:       ${VIRTUALENV_DIR}")
    endif()
  endif()
  if(INSTALL_TO_VIRTUALENV)
    find_program(VIRTUALENV_EXECUTABLE
      NAMES virtualenv
      DOC "The virtualenv executable."
    )
    mark_as_advanced(VIRTUALENV_EXECUTABLE)

    set(VIRTUALENV_DIR  "/opt/ecto/${PROJECT}/${${PROJECT}_GITTAG_SHORT}" CACHE PATH "Virtual environment")

    option(VIRTUALENV_ALL "Add virtualenv to all" off)
    mark_as_advanced(VIRTUALENV_ALL)
    if(VIRTUALENV_ALL)
        set(VIRTUALENV_ALL_ ALL)
    else()
        set(VIRTUALENV_ALL_)
    endif()
    
    add_custom_target(bootstrap ${VIRTUALENV_ALL_})
    add_custom_command(TARGET bootstrap
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/ecto/kitchen/cmake/bootstrap_gen.py
    )
    add_custom_target(virtualenv ${VIRTUALENV_ALL_})
    add_custom_command(TARGET virtualenv
      COMMAND ${PYTHON_EXECUTABLE} bootstrap.py ${VIRTUALENV_DIR}
    )
    
    add_dependencies(virtualenv bootstrap)
    set(CMAKE_INSTALL_PREFIX ${VIRTUALENV_DIR} CACHE PATH "" FORCE)
    set(CMAKE_INSTALL_RPATH  ${VIRTUALENV_DIR}/lib CACHE PATH "" FORCE)
    message(STATUS "virtualenv path:       ${VIRTUALENV_DIR}")
  endif()
endmacro()
