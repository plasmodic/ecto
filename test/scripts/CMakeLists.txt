
macro(pytest _basename)
  message("TEST: ${_basename}")
  add_test(${_basename} ${CMAKE_CURRENT_SOURCE_DIR}/${_basename}.py)
endmacro()

foreach(file
    test_doc
    test_modules
    test_plasm
    test_python_module
    test_reconnect
    test_shared_pass
    test_small_plasms
    test_tendril
    test_tendrils
    )
  pytest(${file})
endforeach()