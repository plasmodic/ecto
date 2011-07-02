#### install stuff #####


#install headers
set(ecto_HEADERS
  include/ecto/ecto.hpp
  include/ecto/except.hpp
  include/ecto/graph_types.hpp
  include/ecto/log.hpp
  include/ecto/module.hpp
  include/ecto/plasm.hpp
  include/ecto/profile.hpp
  include/ecto/registry.hpp
  include/ecto/spore.hpp
  include/ecto/strand.hpp
  include/ecto/tendril.hpp
  include/ecto/tendrils.hpp
  include/ecto/util.hpp
  ${CMAKE_BINARY_DIR}/include/ecto/version.hpp
  )

set(ecto_scheduler_HEADERS
  include/ecto/scheduler/invoke.hpp
  include/ecto/scheduler/singlethreaded.hpp
  include/ecto/scheduler/threadpool.hpp
  )

set (ecto_PYTHON_HEADERS
  include/ecto/python/copy_suite.hpp
  include/ecto/python/raw_constructor.hpp
  include/ecto/python/repr.hpp
  include/ecto/python/std_map_indexing_suite.hpp
  include/ecto/python/std_vector_indexing_suite.hpp
  )

INSTALL(FILES ${ecto_HEADERS}
        DESTINATION ${include_prefix}/ecto
        COMPONENT main
        )
        
INSTALL(FILES ${ecto_scheduler_HEADERS}
        DESTINATION ${include_prefix}/ecto/scheduler
        COMPONENT main
        )
        
INSTALL(FILES ${ecto_PYTHON_HEADERS}
        DESTINATION ${include_prefix}/ecto/python
        COMPONENT main
        )

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
set(ecto_PYTHON_FILES
  python/ecto/__init__.py
  python/ecto/blackbox.py
  python/ecto/doc.py
  python/ecto/module.py
  python/ecto/xdot.py
)
install(FILES ${ecto_PYTHON_FILES}
  DESTINATION ${ecto_PYTHON_INSTALL}/ecto COMPONENT main
  )

configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.inst.in 
  ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
  )

install(FILES ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
        DESTINATION ${share_prefix} COMPONENT main
  )
install(FILES ${CMAKE_SOURCE_DIR}/cmake/python_path.sh.user.in
        DESTINATION ${share_prefix} COMPONENT main
  )

