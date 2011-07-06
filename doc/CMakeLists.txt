# add a target to generate API documentation with Doxygen
#add doc to all
option(BUILD_DOC_ALL "Build the documentation with everything else." OFF)
option(BUILD_DOC_DOXYGEN "Build the c++ api documentation with doxygen, not mutually exclusive." ON)
option(BUILD_DOC_SPHINX "Build the user documentation with sphinx-build, not mutually exclusive." ON)
option(BUILD_DOC_SPHINX_PDF "Build user documentation into a standalone pdf with latex." OFF)
option(INSTALL_DOC "Install the documentation." OFF)

if(BUILD_DOC_ALL)
    message(STATUS "The all target will also build documentation, set BUILD_DOC_ALL to OFF to prevent this.")
    set(build_doc_with_all ALL)
endif()

add_custom_target(doc ${build_doc_with_all}
        COMMENT "Generate all documentation" VERBATIM
        )

if(BUILD_DOC_DOXYGEN)
    #doxygen based docs
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        file(GLOB_RECURSE doc_sources ${CMAKE_SOURCE_DIR}/include/*)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/api
                           COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                           DEPENDS ${doc_sources}
                           )
        add_custom_target(doxygen_doc
            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/api
            COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
        add_dependencies(doc doxygen_doc)
    endif()
endif()

if(BUILD_DOC_SPHINX)
    #sphinx docs
    find_program(SPHINX_BUILD
        sphinx-build
    )
    
    if(SPHINX_BUILD)
        message(STATUS "Found sphinx-build : ${SPHINX_BUILD}")
        file(GLOB_RECURSE rsts ${CMAKE_CURRENT_SOURCE_DIR}/source/*)
        #message(STATUS "sphinx_build deps = ${rsts}")
        add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc
                           COMMAND make html BUILDDIR=${CMAKE_CURRENT_BINARY_DIR}/doc
                           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                           DEPENDS ${rsts}
                           )
        add_custom_target(sphinx_doc
            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doc
            COMMENT "Generating user documentation with Sphinx." VERBATIM
        )
        if(BUILD_DOC_SPHINX_PDF)
            add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/ecto.pdf
                               COMMAND make latexpdf BUILDDIR=${CMAKE_CURRENT_BINARY_DIR}/doc
                               WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                               DEPENDS ${rsts}
                               )
            add_custom_target(sphinx_doc_pdf
                    DEPENDS  ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/ecto.pdf
                    COMMENT "Generating pdf of documentation with Sphinx and latex." VERBATIM
                )
            add_dependencies(doc sphinx_doc_pdf)
            
        endif()

        add_dependencies(doc sphinx_doc)
    endif()
endif()

if(INSTALL_DOC)
    if(BUILD_DOC_DOXYGEN)
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/api
            DESTINATION share/doc/ecto-${ECTO_VERSION}
            COMPONENT doc
            )
    endif()
    if(BUILD_DOC_SPHINX)
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
            DESTINATION share/doc/ecto-${ECTO_VERSION}
            COMPONENT doc
            )
    endif()
    if(BUILD_DOC_SPHINX_PDF)
        install(FILES  ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/ecto.pdf
            DESTINATION share/doc/ecto-${ECTO_VERSION}
            COMPONENT doc
            )
    endif()
endif()

