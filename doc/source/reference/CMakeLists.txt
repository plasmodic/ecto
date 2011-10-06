add_executable(tutorial_native_plasm
  native_plasm.cpp
  )

target_link_libraries(tutorial_native_plasm
  ecto
  ${ECTO_DEP_LIBS}
  hello_ecto_ectomodule #uses cells from this module.
)