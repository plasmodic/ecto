find_package(Boost COMPONENTS
  python
  )

find_package(PythonLibs)

include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  )

set(ECTO_DEP_LIBS ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

