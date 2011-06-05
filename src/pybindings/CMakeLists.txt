ectomodule(ecto
  ecto.cpp
  plasm.cpp
  tendril.cpp
  tendrils.cpp
  module.cpp
  schedulers.cpp
  )

set_target_properties(ecto_ectomodule PROPERTIES
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/share/ecto/python"
  )

#this is the python extension
install(TARGETS ecto_ectomodule
  LIBRARY DESTINATION ${ecto_PYTHON_INSTALL} COMPONENT ecto_python
  )

