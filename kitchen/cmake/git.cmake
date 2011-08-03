find_package(Git)

macro(git_status PROJECT)
  if (GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} log -n1 --pretty=format:%H
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_COMMITHASH
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_COMMITHASH_STATUS
      )

    execute_process(COMMAND ${GIT_EXECUTABLE} log -n1 --pretty=format:%cD
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_LAST_MODIFIED
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_TREEHASH_STATUS
      )

    execute_process(COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_GITTAG
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_GITTAG_STATUS
      )

  else()
    set(${PROJECT}_COMMITHASH treehash_unavailable)
    set(${PROJECT}_LAST_MODIFIED lastmod_unavailable)
    set(${PROJECT}_GITTAG   tag_unavailable)
  endif()

  message(STATUS "${PROJECT} commit:       ${${PROJECT}_COMMITHASH}")
  message(STATUS "${PROJECT} tag:          ${${PROJECT}_GITTAG}")
  message(STATUS "${PROJECT} last_mod:     ${${PROJECT}_LAST_MODIFIED}")


endmacro()

