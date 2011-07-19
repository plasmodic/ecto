include_directories(${CMAKE_CURRENT_SOURCE_DIR})
ectomodule(ecto
  ecto.cpp
  except.cpp
  plasm.cpp
  tendril.cpp
  tendrils.cpp
  cell.cpp
  schedulers.cpp
  strand.cpp
  cells/Constant.cpp
  cells/Passthrough.cpp
  )

set_target_properties(ecto_ectomodule PROPERTIES
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/share/ecto/python"
  )

#this is the python extension
install(TARGETS ecto_ectomodule
  DESTINATION ${ecto_PYTHON_INSTALL}
  COMPONENT main
  )

