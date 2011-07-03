set(ecto_PYTHON_FILES
  __init__.py
  blackbox.py
  doc.py
  module.py
  xdot.py
)
install(FILES ${ecto_PYTHON_FILES}
  DESTINATION ${ecto_PYTHON_INSTALL}/ecto COMPONENT main
  )