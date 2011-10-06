if(NOT ecto_kitchen_SOURCE_DIR)
    include(doc.cmake)

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

    find_package(Doxygen)
    if(DOXYGEN_FOUND)
      set(DOC_SEARCH_DIRS
        ${ecto_SOURCE_DIR}/include
      )
      doxygen(doxygen ${DOC_SEARCH_DIRS})
      add_dependencies(doc doxygen)
    endif()

    find_sphinx()
    if(SPHINX_BUILD)
      sphinx(html ${CMAKE_CURRENT_SOURCE_DIR}/source  ${CMAKE_CURRENT_BINARY_DIR}/html
        ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
        ${ecto_SOURCE_DIR}/python
        ${CMAKE_CURRENT_BINARY_DIR}
        )
      add_dependencies(doc html)
      if(DOXYGEN_FOUND)
        add_dependencies(html doxygen)
      endif()

      deploy(www-deploy ${CMAKE_CURRENT_BINARY_DIR}/html/ ${ECTO_DOC_DEPLOY_DESTINATION})
      add_dependencies(www-deploy doc)
    endif()
endif()

#these are used by the docs.
add_subdirectory(source doc_errata)
