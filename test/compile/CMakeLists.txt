
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

add_compile_fail_test(redeclare)


