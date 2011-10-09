#### install stuff #####

#create an ectoConfig.cmake for easy find_package(ecto)
set(ecto_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${include_prefix})
set(ecto_LIBRARIES_DIR ${CMAKE_INSTALL_PREFIX}/lib)

if(UNIX)
  set(ecto_LIBRARIES ${ecto_LIBRARIES_DIR}/libecto.so)
elseif(WIN32)
  set(ecto_LIBRARIES ${ecto_LIBRARIES_DIR}/ecto_cpp.dll)
endif()

#FIXME make the python install path reflect the version for side by side...
set(ecto_PYTHONPATH ${CMAKE_INSTALL_PREFIX}/${ecto_PYTHON_INSTALL})
set(ECTO_CONFIG_PATH  ${CMAKE_INSTALL_PREFIX}/${share_prefix})
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig-version.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig-version.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoMacros.cmake
  ${CMAKE_BINARY_DIR}/unix_install/ectoMacros.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/rosbuild_lite.cmake
  ${CMAKE_BINARY_DIR}/unix_install/rosbuild_lite.cmake @ONLY)

#install the ectoConfig.cmake and ectoConfig-version.cmake
install(DIRECTORY
  ${CMAKE_BINARY_DIR}/unix_install/   #last component empty, so we loose the unix_install
  DESTINATION ${share_prefix}
  COMPONENT main
  )

install(FILES ${ecto_SOURCE_DIR}/cmake/python_path.sh.user.in
  DESTINATION ${share_prefix}
  COMPONENT main
  )

#regular headers
install(DIRECTORY ${ecto_SOURCE_DIR}/include/ecto
  DESTINATION ${include_prefix}
  COMPONENT main
  )

#generated headers
install(DIRECTORY ${CMAKE_BINARY_DIR}/include/ecto
  DESTINATION ${include_prefix}
  COMPONENT main
  )

#checkinstall --exclude=/home -y --nodoc --pkgname ecto-amoeba <<EOF
#Ecto, a hybrid c++/python framework for dataflow pipeline development.
#EOF


add_custom_target(checkinstall
  COMMENT "Install using checkinstall." VERBATIM
  )

add_custom_command(TARGET checkinstall
  COMMAND sh -c "sudo checkinstall --exclude=/home -y --nodoc --pkgname ecto-${ECTO_CODE_NAME} <<EOF Ecto, a hybrid c++/python framework for dataflow pipeline development. EOF"
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
