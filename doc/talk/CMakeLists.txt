
set(outdir ${CMAKE_BINARY_DIR}/doc/html/talk)
file(MAKE_DIRECTORY ${outdir})

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/talk.html
  COMMAND rsync -va ${CMAKE_CURRENT_SOURCE_DIR}/ui/ ${outdir}/ui/
  COMMAND cp ${ecto_SOURCE_DIR}/kitchen/doc/source/art/* ${outdir}
  COMMAND rst2s5 --theme-url=ui/ecto ${CMAKE_CURRENT_SOURCE_DIR}/talk.rst ${outdir}/talk.html
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
add_custom_target(talk-html DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/talk.html)