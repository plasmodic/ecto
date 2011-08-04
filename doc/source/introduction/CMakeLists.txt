add_executable(functor functor.cpp)

ectomodule(introduction
  module.cpp
  cell.cpp
  nop.cpp
  interface.cpp
  )

add_dependencies(html 
  introduction_ectomodule functor
  )



