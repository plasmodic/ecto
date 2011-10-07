set(ECTO_MAJOR_VERSION 0)
set(ECTO_MINOR_VERSION 1)
set(ECTO_PATCH_VERSION 0)
set(ECTO_SOVERSION ${ECTO_MAJOR_VERSION}.${ECTO_MINOR_VERSION})
set(ECTO_VERSION ${ECTO_MAJOR_VERSION}.${ECTO_MINOR_VERSION}.${ECTO_PATCH_VERSION})
set(ECTO_CODE_NAME "amoeba") #code name must be hand coded for debian to work, unless something more clever exists


set(include_prefix include/ecto-${ECTO_CODE_NAME})
set(share_prefix share/ecto-${ECTO_CODE_NAME})

configure_file(${ecto_SOURCE_DIR}/cmake/version.hpp.in ${CMAKE_BINARY_DIR}/include/ecto/version.hpp)
include_directories(${CMAKE_BINARY_DIR}/include)
