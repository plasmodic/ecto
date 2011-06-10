
macro(pytest testname)
  add_test(${testname}
    /usr/bin/env
    PYTHONPATH=${LIBRARY_OUTPUT_PATH}:${CMAKE_SOURCE_DIR}/python:$PYTHONPATH
    ${ARGN}
    )
endmacro()

pytest(test_lineplasm0 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 1 1 1 50)
pytest(test_lineplasm1 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 1 5 5 50)
pytest(test_lineplasm2 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 10 10 10 50)
pytest(test_lineplasm3 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 15 25 35 50)


foreach(file
    test_doc
    test_modules
    test_parameter_callbacks
    test_plasm
    test_python_module
    test_reconnect
    test_shared_pass
    test_small_plasms
    test_tendril
    test_tendrils
    )
  add_test(${file} 
    /usr/bin/env
    PYTHONPATH=${LIBRARY_OUTPUT_PATH}:${CMAKE_SOURCE_DIR}/python:$PYTHONPATH
    ${CMAKE_CURRENT_SOURCE_DIR}/${file}.py)
endforeach()

