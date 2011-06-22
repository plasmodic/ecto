cmake_minimum_required(VERSION 2.8)

project(ecto_samples)
    
#find_package(ecto REQUIRED)
    
ectomodule(hello_ecto
    hello_ecto.cpp
)
#optionally link against other libs
#ecto_link(hello_ecto
#  ${MY_EXTRA_LIBS}
#)

#set_ecto_install_package_name(ecto_samples)

#generates an install target for your module (puts it in the python dist folder)
install_ecto_module(hello_ecto)

#install the main shared lib
install(PROGRAMS ecto_doc.py
  DESTINATION bin
  COMPONENT ecto_python
  )
