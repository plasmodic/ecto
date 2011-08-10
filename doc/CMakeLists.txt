
add_custom_command(OUTPUT gitstats.txt
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/make_gitstats.py 
  ${ecto_SOURCE_DIR} -o ${CMAKE_CURRENT_BINARY_DIR}/gitstats.txt)

add_custom_target(gitstats DEPENDS gitstats.txt)
  

add_subdirectory(talk)