set (ecto_PYTHON_HEADERS
  copy_suite.hpp
  raw_constructor.hpp
  repr.hpp
  std_map_indexing_suite.hpp
  std_vector_indexing_suite.hpp
  )
       
INSTALL(FILES ${ecto_PYTHON_HEADERS}
        DESTINATION ${include_prefix}/ecto/python
        COMPONENT main
        )
