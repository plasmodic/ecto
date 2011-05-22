#### install stuff #####

#install headers
set(ecto_HEADERS
  include/ecto/ecto.hpp
  include/ecto/module.hpp
  include/ecto/plasm.hpp
  include/ecto/tendril.hpp
  include/ecto/tendrils.hpp
  include/ecto/util.hpp
  )
  
set (ecto_PYTHON_HEADERS
  include/ecto/python/copy_suite.hpp
  include/ecto/python/raw_constructor.hpp
  include/ecto/python/repr.hpp
  include/ecto/python/std_map_indexing_suite.hpp
  include/ecto/python/std_vector_indexing_suite.hpp
  )

INSTALL(FILES ${ecto_HEADERS}
  DESTINATION include/ecto
  COMPONENT ecto
  )
  
INSTALL(FILES ${ecto_PYTHON_HEADERS}
  DESTINATION include/ecto/python
  COMPONENT ecto
  )
  
#create an ectoConfig.cmake for easy find_package(ecto)
set(ecto_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(ecto_LIBRARIES ecto)
set(ecto_LIBRARIES_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(ecto_PYTHON_INSTALL ${PYTHON_PACKAGES_PATH} )
set(ecto_PYTHONPATH ${CMAKE_INSTALL_PREFIX}/${ecto_PYTHON_INSTALL})
configure_file(${CMAKE_SOURCE_DIR}/cmake/ectoConfig.cmake.in 
  ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake @ONLY)

#install the ectoConfig.cmake
INSTALL(FILES ${CMAKE_BINARY_DIR}/unix_install/ectoConfig.cmake
  DESTINATION share/ecto
  COMPONENT ecto
  )

#install python stuff


#python support
set(ecto_PYTHON_FILES
  python/ecto/__init__.py
  python/ecto/doc.py
  python/ecto/module.py
  python/ecto/xdot.py
)

          
install(FILES ${ecto_PYTHON_FILES}
  DESTINATION ${ecto_PYTHON_INSTALL}/ecto COMPONENT ecto_python
  )

configure_file(${CMAKE_SOURCE_DIR}/cmake/python_path.sh.inst.in 
  ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
  )

install(FILES ${CMAKE_BINARY_DIR}/unix_install/python_path.sh
        DESTINATION share/ecto COMPONENT ecto_python
  )
install(FILES ${CMAKE_SOURCE_DIR}/cmake/python_path.sh.user.in
        DESTINATION share/ecto COMPONENT ecto_python
  )
