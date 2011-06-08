find_package(Threads)

find_package(Boost COMPONENTS
  python
  thread
  system
  REQUIRED
  )

find_package(PythonLibs REQUIRED)

include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  )

set(ECTO_DEP_LIBS 
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

#detect the python version and install directories
find_package(PythonInterp REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE} --version
          ERROR_VARIABLE PYTHON_VERSION_FULL
          OUTPUT_STRIP_TRAILING_WHITESPACE)
          
string(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")

if(UNIX)
    if(APPLE)
        set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages CACHE PATH "Where to install the python packages.")
    else() #debian based assumed, install to the dist-packages.
        set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/dist-packages CACHE PATH "Where to install the python packages.")
    endif()
elseif(WIN32)
    get_filename_component(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    set(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages" CACHE PATH "Where to install the python packages.")
endif()
