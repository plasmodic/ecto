# add a target to generate API documentation with Doxygen
# add doc to all

project(kitchen_doc)

#we're not in the kitchen FIXME hack for standalone
if(NOT ecto_kitchen_SOURCE_DIR)
  add_custom_target(html)
  add_custom_target(pdf)
  add_custom_target(latex)
  MESSAGE(WARNING "Documentation only supported in the kitchen, will be fixed soon.")
  return()
endif()

option(INSTALL_DOC "Install the documentation." OFF)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Makefile.sphinx.in 
  ${CMAKE_CURRENT_BINARY_DIR}/Makefile.sphinx
  @ONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ecto_kitchen_config.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/ecto_kitchen_config.py
  @ONLY)

add_custom_target(doc
  COMMENT "Generate all documentation" VERBATIM
  )

#doxygen based docs
find_package(Doxygen)
if(DOXYGEN_FOUND)
  file(GLOB_RECURSE doc_sources ${CMAKE_SOURCE_DIR}/include/*)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

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
  #dependencies for sphinx, html must be built for the sphinx to not error out.
  #also all modules should be built so that the sphinx ectodoc stuff works.
  add_dependencies(html ecto ecto_ectomodule ecto_test_ectomodule)
  add_dependencies(pdf ecto ecto_ectomodule ecto_test_ectomodule)
  add_dependencies(html doxygen)
  add_dependencies(pdf doxygen)
  add_dependencies(doc pdf)
  add_dependencies(doc html)

endif()

if(INSTALL_DOC)
  if(DOXYGEN_FOUND AND SPHINX_BUILD)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
      DESTINATION share/doc/ecto-${ECTO_VERSION}
      COMPONENT doc
      )
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/latex/ecto.pdf
      DESTINATION share/doc/ecto-${ECTO_VERSION}
      COMPONENT doc
      )
  endif()
endif()

