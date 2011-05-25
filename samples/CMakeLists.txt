#cmake_minimum_required(VERSION 2.8)
project(ecto_samples)

set(BUILD_SAMPLES ON CACHE BOOL "Build sample modules")

#ectomodule(hello_ecto
#    hello_ecto.cpp
#)

#install the main shared lib
install(PROGRAMS ecto_doc.py
  DESTINATION bin
  COMPONENT ecto_python
  )
