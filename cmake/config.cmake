set(ecto_CONFIG_DIR ${ecto_BINARY_DIR}/config)

set(ecto_PYTHONPATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${ecto_SOURCE_DIR}/python)
set(ecto_INCLUDE_DIRS ${ecto_SOURCE_DIR}/include ${ecto_BINARY_DIR}/include)
set(ecto_PYTHONLIB ecto_ectomodule)

if(UNIX)
  #don't put the soname on this guy, as it confuses the linker at runtime.
  set(ecto_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libecto.so) 
elseif(WIN32)
  set(ecto_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto_cpp.dll)
endif()

set(ECTO_CONFIG_PATH  ${ecto_SOURCE_DIR}/cmake)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig.cmake.in
  ${ecto_CONFIG_DIR}/ectoConfig.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig-version.cmake.in
  ${ecto_CONFIG_DIR}/ectoConfig-version.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoMacros.cmake
  ${ecto_CONFIG_DIR}/ectoMacros.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/rosbuild_lite.cmake
  ${ecto_CONFIG_DIR}/rosbuild_lite.cmake @ONLY)

#copy the python_path.sh.user.in for users that are not installing ecto.
file(COPY ${PROJECT_SOURCE_DIR}/cmake/python_path.sh.user.in DESTINATION ${ecto_CONFIG_DIR})
file(COPY ${PROJECT_SOURCE_DIR}/cmake/doc.cmake DESTINATION ${ecto_CONFIG_DIR})
file(COPY ${PROJECT_SOURCE_DIR}/cmake/git.cmake DESTINATION ${ecto_CONFIG_DIR})
#set this back for our libs to pick it up as
set(ecto_LIBRARIES ecto)
