find_package(Boost COMPONENTS
  python
  REQUIRED
  )

find_package(PythonLibs REQUIRED)

include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  )

set(ECTO_DEP_LIBS ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

#detect the python version and install directories
find_package(PythonInterp REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE} --version
          ERROR_VARIABLE PYTHON_VERSION_FULL
          OUTPUT_STRIP_TRAILING_WHITESPACE)
          
string(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")

if(UNIX)
    set(PYTHON_PLUGIN_INSTALL_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages/opencv)
    set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages)
elseif(WIN32)
    get_filename_component(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    set(PYTHON_PLUGIN_INSTALL_PATH "${PYTHON_PATH}/Lib/site-packages/opencv")
    set(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages")
endif()
