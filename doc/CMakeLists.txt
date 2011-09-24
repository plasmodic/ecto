macro(init_docs PROJECT CODE_NAME)

endmacro()

# add a target to generate API documentation with Doxygen
# add doc to all
set(ECTO_DOC_DEPLOY_DESTINATION_BASE /tmp
    CACHE PATH "Base path for deploying docs"
    )

set(ECTO_DOC_DEPLOY_DESTINATION ${ECTO_DOC_DEPLOY_DESTINATION_BASE}/${${PROJECT_NAME}_GITTAG_SHORT}
    CACHE PATH "Destination for deploying docs"
    )

set(DOC_SOURCE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ecto_SOURCE_DIR}/samples
    ${ecto_SOURCE_DIR}
    )

configure_file(doc_config.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/doc_config.py
)

add_custom_target(doc
  COMMENT "Generate all documentation" VERBATIM
  )

#doxygen based docs
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOC_SEARCH_DIRS
    ${ecto_SOURCE_DIR}/include
  )
  #TODO FIXME the sources for doxygen are a bit messed up.
  foreach(dir ${DOC_SEARCH_DIRS})
    file(GLOB_RECURSE _doc_sources ${dir}/*)
    list(APPEND doc_sources ${_doc_sources})
  endforeach()
  string(REPLACE ";" " " doc_sources "${doc_sources}")
  configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  #TODO FIXME the sources for doxygen are a bit messed up.
  add_custom_target(doxygen
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

  add_custom_command(TARGET doxygen
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_dependencies(doc doxygen)
endif()

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

  add_custom_target(html)
  add_custom_command(TARGET html
    COMMAND
    /usr/bin/env
    PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:${ecto_SOURCE_DIR}/python:${CMAKE_CURRENT_BINARY_DIR}
    sphinx-build  ${CMAKE_CURRENT_SOURCE_DIR}/source ${CMAKE_CURRENT_BINARY_DIR}/html
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_dependencies(html doxygen)
  add_dependencies(doc html)
  add_custom_target(www-deploy)
  add_custom_command(TARGET www-deploy
    COMMAND rsync --perms --chmod=a+rX -va  ${CMAKE_CURRENT_BINARY_DIR}/html/ ${ECTO_DOC_DEPLOY_DESTINATION}
  )
  add_dependencies(www-deploy doc)
endif()


