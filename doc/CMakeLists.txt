# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
if(NOT ecto_kitchen_SOURCE_DIR)
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

