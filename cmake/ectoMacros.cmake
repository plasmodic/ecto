macro(ectomodule NAME)

  add_library(${NAME}_ectomodule SHARED
    ${ARGN}
    )

  set_target_properties(${NAME}_ectomodule
    PROPERTIES
    OUTPUT_NAME ${NAME}
    PREFIX ""
    )

  target_link_libraries(${NAME}_ectomodule
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ecto
    )
endmacro()

macro(link_ecto NAME)
  target_link_libraries(${NAME}_ectomodule
    ${ARGN}
  )
endmacro()

macro( install_ecto_module name)
#this is the python extension
install(TARGETS ${name}_ectomodule
  LIBRARY DESTINATION ${ecto_module_PYTHON_INSTALL} COMPONENT main
  )
endmacro()

