#add_executable(functor functor.cpp)

ectomodule(ecto_overview
  module.cpp
  cell.cpp
  nop.cpp
  interface.cpp
  )

if(SPHINX_BUILD)
  add_dependencies(html 
    ecto_overview_ectomodule functor
    )
endif()