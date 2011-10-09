
##
# doxygen(<TARGET_NAME> <SEARCH_DIRS>)
# TARGET_NAME -> The cmake target to create.
# SEARCH_DIRS -> a CMake List of directories to search for doxygenated files.
#
macro(doxygen TARGET_NAME SEARCH_DIRS)
  #doxygen based docs
  set(DOC_SEARCH_DIRS ${SEARCH_DIRS}
  )
  foreach(dir ${DOC_SEARCH_DIRS})
    file(GLOB_RECURSE _doc_sources ${dir}/*)
    list(APPEND doc_sources ${_doc_sources})
  endforeach()

  string(REPLACE ";" " " doc_sources "${doc_sources}")

  configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

  add_custom_target(${TARGET_NAME}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

  add_custom_command(TARGET ${TARGET_NAME}
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_dependencies(doc ${TARGET_NAME})
endmacro()

macro(find_sphinx)
  find_program(SPHINX_BUILD sphinx-build)
  if(SPHINX_BUILD)
    set(REQUIRED_SPHINX_VERSION "1.0.7")
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sphinx;print sphinx.__version__"
      OUTPUT_VARIABLE SPHINX_VERSION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    if("${SPHINX_VERSION}" VERSION_LESS ${REQUIRED_SPHINX_VERSION})
      MESSAGE(WARNING "You version of sphinx (http://sphinx.pocoo.org) is ${SPHINX_VERSION}, required ${REQUIRED_SPHINX_VERSION}")
      if (UNIX)
        MESSAGE(WARNING "You may be able to update with 'easy_install -U sphinx'")
      endif()
    endif()
  endif()
endmacro()

##
# sphinx(<TARGET_NAME> <SOURCE_DIR> <BUILD_DIR> [PATH1 [ PATH2 [ PATH3 ]]])
# TARGET_NAME -> The cmake target to create.
# SOURCE_DIR -> Where the conf.py is
# BUILD_DIR -> Where should sphinx put the result
# PATH(s) -> paths to prepend to the PYTHONPATH for the execution of sphinx-build
#
macro(sphinx TARGET_NAME SOURCE_DIR BUILD_DIR)
  set(_PYTHONPATH "")
  foreach(path ${ARGN})
    set(_PYTHONPATH "${_PYTHONPATH}:${path}")#first come first serve
  endforeach()
  set(_PYTHONPATH "${_PYTHONPATH}:$ENV{PYTHONPATH}")#append the environment python path
  
  add_custom_target(${TARGET_NAME})
  add_custom_command(TARGET ${TARGET_NAME}
    COMMAND
    /usr/bin/env
    PYTHONPATH=${_PYTHONPATH}
    ${SPHINX_BUILD} -aE ${SOURCE_DIR} ${BUILD_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endmacro()

##
# deploy(<TARGET_NAME> <DOCS> <DESTINATION>)
# using rsync create a target that deploys the docs
#
macro(deploy TARGET_NAME DOCS DESTINATION)
  add_custom_target(${TARGET_NAME})
  add_custom_command(TARGET ${TARGET_NAME}
    COMMAND rsync --perms --chmod=a+rX -va  ${DOCS} ${DESTINATION}
  )
endmacro()

