set(ecto_PYTHONPATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${ecto_SOURCE_DIR}/python)
set(ecto_INCLUDE_DIRS ${ecto_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)

if(UNIX)
  #don't put the soname on this guy, as it confuses the linker at runtime.
  set(ecto_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libecto.so) 
elseif(WIN32)
  set(ecto_LIBRARIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto_cpp.dll)
endif()

set(ECTO_CONFIG_PATH  ${ecto_SOURCE_DIR}/cmake)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig.cmake.in
  ${CMAKE_BINARY_DIR}/ectoConfig.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoConfig-version.cmake.in
  ${CMAKE_BINARY_DIR}/ectoConfig-version.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/ectoMacros.cmake
  ${CMAKE_BINARY_DIR}/ectoMacros.cmake @ONLY)
configure_file(${ecto_SOURCE_DIR}/cmake/rosbuild_lite.cmake
  ${CMAKE_BINARY_DIR}/rosbuild_lite.cmake @ONLY)

#set this back for our libs to pick it up as
set(ecto_LIBRARIES ecto)
