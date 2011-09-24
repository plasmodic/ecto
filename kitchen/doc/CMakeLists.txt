# add a target to generate API documentation with Doxygen
# add doc to all

set(ECTO_DOC_DEPLOY_DESTINATION_BASE /tmp
    CACHE PATH "Base path for deploying docs"
    )

set(ECTO_DOC_DEPLOY_DESTINATION ${ECTO_DOC_DEPLOY_DESTINATION_BASE}/${${PROJECT_NAME}_GITTAG_SHORT}
    CACHE PATH "Destination for deploying docs"
    )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Makefile.sphinx.in 
  ${CMAKE_CURRENT_BINARY_DIR}/Makefile.sphinx
  @ONLY
  )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ecto_kitchen_config.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/ecto_kitchen_config.py
#  @ONLY
)

add_custom_target(doc
  COMMENT "Generate all documentation" VERBATIM
  )

#doxygen based docs
find_package(Doxygen)
if(DOXYGEN_FOUND)
  
  #TODO FIXME the sources for doxygen are a bit messed up.
  foreach(proj ${KITCHEN_PROJECTS})
    file(GLOB_RECURSE _doc_sources ${CMAKE_SOURCE_DIR}/${proj}/include/*)
    list(APPEND doc_sources ${_doc_sources})
  endforeach()
  string(REPLACE ";" " " doc_sources "${doc_sources}")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
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
  message(STATUS "Found sphinx-build : ${SPHINX_BUILD}")
  message(STATUS "sphinx version: ${SPHINX_VERSION}")
  if("${SPHINX_VERSION}" VERSION_LESS ${REQUIRED_SPHINX_VERSION})
    MESSAGE(WARNING "You version of sphinx (http://sphinx.pocoo.org) is ${SPHINX_VERSION}, required ${REQUIRED_SPHINX_VERSION}")
    if (UNIX)
      MESSAGE(WARNING "You may be able to update with 'easy_install -U sphinx'")
    endif()
  endif()
  set(ENV{PYTHONPATH} @LIBRARY_OUTPUT_PATH@:@CMAKE_SOURCE_DIR@/python:$ENV{PYTHONPATH})
  add_custom_target(pdf
    COMMENT "Generating pdf of documentation with Sphinx and latex." VERBATIM
    )
  add_custom_command(TARGET pdf
    COMMAND ${CMAKE_MAKE_PROGRAM} -f Makefile.sphinx latexpdf
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_target(html)
  add_custom_command(TARGET html
    COMMAND ${CMAKE_MAKE_PROGRAM} -f Makefile.sphinx html
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )

  add_custom_target(www-deploy)
  add_custom_command(TARGET www-deploy
    COMMAND ${CMAKE_MAKE_PROGRAM} -f Makefile.sphinx deploy
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )

  #dependencies for sphinx, html must be built for the sphinx to not error out.
  #also all modules should be built so that the sphinx ectodoc stuff works.
  add_dependencies(html ecto ecto_ectomodule ecto_test_ectomodule)
  add_dependencies(pdf ecto ecto_ectomodule ecto_test_ectomodule)
  add_dependencies(html doxygen)
  add_dependencies(pdf doxygen)
  add_dependencies(doc pdf)
  add_dependencies(doc html)

  #disable this install because the docs aren't always built.
  #install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
  #  DESTINATION share/doc/${${PROJECT_NAME}_GITTAG_SHORT}
  #  COMPONENT doc
  #)
endif()


