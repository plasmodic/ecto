macro(pytest testname)
  add_test(ecto_${testname}
    /usr/bin/env
    PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:${ecto_SOURCE_DIR}/python:$ENV{PYTHONPATH}
    ${ARGN}
    )
endmacro()

foreach(file
    throw_in_interpreter_thread

    test_addergraph
    test_async_bp_objects
    test_async_execution
    test_async_execution_harsh
    test_async_multiple_sched
    test_async
    test_blackbox
    #test_bp_to_cell_ptr
    test_constant
    test_dealer
    test_doc
    test_dual_line_plasm
    test_entanglement
    test_exception
    test_exception_in_constructor
    test_fileIO
    test_handles
    test_If
    test_metrics
    test_module_qualification
    test_modules
    test_multiprocess
    test_no_ecto_import
    test_options
    test_parameter_callbacks
    test_passthrough
    test_plasm
    test_process_return_values
    test_python_module
    test_random
    test_reconnect
    test_redirect
    test_required_io
    test_required_param
    # test_restart  # this needs thinking about... can't test nexecutions with a stateful cell
    test_schedbasics
    test_shared_pass
    test_small_plasms
    test_start_stop_counter
    test_strands
    test_tendril
    test_tendrils
    test_throw
    test_type_mismatch_errors
    test_wrong_param_type
    )
    add_test(ecto_${file}
        /usr/bin/env
        PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:${ecto_SOURCE_DIR}/python:$PYTHONPATH
        ${CMAKE_CURRENT_SOURCE_DIR}/${file}.py)

endforeach()

pytest(test_lineplasm0 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 1 1 1 50)
pytest(test_lineplasm1 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 1 5 5 50)
pytest(test_lineplasm2 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 10 10 10 50)
# pytest(test_lineplasm3 ${CMAKE_CURRENT_SOURCE_DIR}/test_lineplasm.py 15 25 35 50)

pytest(save_plasm_to_file
  ${CMAKE_CURRENT_SOURCE_DIR}/save_plasm_to_file.py
  ${ecto_BINARY_DIR}/plasm.ecto)

pytest(load_plasm_from_file
  ${CMAKE_CURRENT_SOURCE_DIR}/save_plasm_to_file.py
  ${ecto_BINARY_DIR}/plasm.ecto)
