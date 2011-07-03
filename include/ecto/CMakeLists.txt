set(ecto_HEADERS
  cell.hpp
  ecto.hpp
  except.hpp
  graph_types.hpp
  log.hpp
  plasm.hpp
  profile.hpp
  registry.hpp
  spore.hpp
  strand.hpp
  tendril.hpp
  tendrils.hpp
  util.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/version.hpp
)

INSTALL(FILES ${ecto_HEADERS}
        DESTINATION ${include_prefix}/ecto
        COMPONENT main
        )