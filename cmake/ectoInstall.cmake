#### install stuff #####

#create an ectoConfig.cmake for easy find_package(ecto)
set(ecto_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${include_prefix})
set(ecto_LIBRARIES_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(ecto_LIBRARIES ${ecto_LIBRARIES_DIR}/libecto.so.${ECTO_VERSION})

#FIXME make the python install path reflect the version for side by side...
set(ecto_PYTHONPATH ${CMAKE_INSTALL_PREFIX}/${ecto_PYTHON_INSTALL})
set(ECTO_CONFIG_PATH  ${CMAKE_INSTALL_PREFIX}/${share_prefix})
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig-version.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig-version.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoMacros.cmake
  ${CMAKE_BINARY_DIR}/unix_install/ectoMacros.cmake @ONLY)
    
#install the ectoConfig.cmake and ectoConfig-version.cmake
INSTALL(FILES
  ${CMAKE_BINARY_DIR}/unix_install/ectoMacros.cmake
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig-version.cmake
  DESTINATION ${ECTO_CONFIG_PATH}
  COMPONENT main
  )

#install python stuff
#python support
configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.inst.in 
  ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
  )

install(FILES ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
        DESTINATION ${share_prefix} COMPONENT main
  )
install(FILES ${CMAKE_SOURCE_DIR}/cmake/python_path.sh.user.in
        DESTINATION ${share_prefix} COMPONENT main
  )

add_custom_target(checkinstall
            COMMAND checkinstall -y --pkgname=ecto-${ECTO_VERSION} make install
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "checkinstall." VERBATIM
        )
