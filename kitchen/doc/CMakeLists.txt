# add a target to generate API documentation with Doxygen
# add doc to all

option(INSTALL_DOC "Install the documentation." OFF)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Makefile.in 
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
  message(STATUS "Found sphinx-build : ${SPHINX_BUILD}")
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
  add_dependencies(html ecto_cpp ecto_ectomodule ecto_test_ectomodule)
  add_dependencies(pdf ecto_cpp ecto_ectomodule ecto_test_ectomodule)
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

