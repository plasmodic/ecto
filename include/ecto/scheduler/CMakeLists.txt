set(ecto_scheduler_HEADERS
  invoke.hpp
  singlethreaded.hpp
  threadpool.hpp
  )

INSTALL(FILES ${ecto_scheduler_HEADERS}
        DESTINATION ${include_prefix}/ecto/scheduler
        COMPONENT main
        )
