#create a soft link to our __init__.py
#TODO FIXME is this supported on other platforms?
execute_process(COMMAND mkdir -p ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto)
execute_process(COMMAND ln -sf ecto/__init__.py
  WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto
  )

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ecto
  DESTINATION ${ecto_PYTHON_INSTALL}
  COMPONENT main
  FILES_MATCHING PATTERN "*.py"
)
