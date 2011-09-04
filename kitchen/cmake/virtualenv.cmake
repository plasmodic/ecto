macro(setup_virtualenv PROJECT)
  option(INSTALL_TO_VIRTUALENV ON)
  
  if(INSTALL_TO_VIRTUALENV)
    find_program(VIRTUALENV_EXECUTABLE
      NAMES virtualenv
      DOC "The virtualenv executable."
    )
    mark_as_advanced(VIRTUALENV_EXECUTABLE)

    set(VIRTUALENV_DIR  "/opt/ecto/${PROJECT}/${${PROJECT}_GITTAG_SHORT}" CACHE PATH "Virtual environment")

    add_custom_target(
      boostrap
    )
    add_custom_command(TARGET boostrap
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/ecto/kitchen/cmake/bootstrap_gen.py
      DEPENDS ${CMAKE_SOURCE_DIR}/ecto/kitchen/cmake/bootstrap_gen.py
    )

    add_custom_target(virtualenv)
    add_custom_command(TARGET virtualenv
      COMMAND ${PYTHON_EXECUTABLE} bootstrap.py ${VIRTUALENV_DIR}
    )
    add_dependencies(virtualenv bootstrap)
    
    set(CMAKE_INSTALL_PREFIX ${VIRTUALENV_DIR} CACHE PATH "" FORCE)
    set(CMAKE_INSTALL_RPATH  ${VIRTUALENV_DIR}/lib CACHE PATH "" FORCE)
    message(STATUS "virtualenv path:       ${VIRTUALENV_DIR}")
  endif()
endmacro()
