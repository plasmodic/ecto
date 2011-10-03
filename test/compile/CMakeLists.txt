
macro(add_compile_fail_test FILENAME)
  add_library(compile_fail_${FILENAME} 
    SHARED
    EXCLUDE_FROM_ALL
    ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.cpp
    )

  add_test(NAME compile_fail_${FILENAME}
    COMMAND
    ${CMAKE_CURRENT_SOURCE_DIR}/fail.sh 
    ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_CURRENT_BINARY_DIR} compile_fail_${FILENAME})

endmacro()

macro(add_compile_pass_test FILENAME)
  add_library(compile_pass_${FILENAME} 
    SHARED
    EXCLUDE_FROM_ALL
    ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.cpp
    )

  add_test(NAME compile_pass_${FILENAME}
    COMMAND
    ${CMAKE_CURRENT_SOURCE_DIR}/pass.sh 
    ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_CURRENT_BINARY_DIR} compile_pass_${FILENAME})

endmacro()

add_compile_fail_test(redeclare)
#TODO need to add expected error here?
add_compile_pass_test(tendril_static_time_correct)
add_compile_pass_test(tendril_static_time_correct2)
add_compile_fail_test(tendril_static_time_nonspore)

