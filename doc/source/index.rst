ecto doc
================================
ecto is...

Contents:

.. toctree::
   :maxdepth: 2

Building
================================
Using a standard cmake build system:

.. code-block:: sh
  
  mkdir build
  cd build
  cmake ..
  make
  

The libs will be located in the build folder under lib

Assuming you're in the top level ecto directory, try the following in a shell:

.. code-block:: sh

  #add ecto to your python path
  . build/python_path.sh
  python test/python/test_plasm.py
  
Structure
================================
  - include
      c++ include files
  - src/lib
      Mostly straight implementation, with little boost::python, results in libecto.so
  - src/pybindings
      The python bindings for libecto.so, results in the python module ecto.
  - python
      ecto python library, has a few utilies that are available in python.
  - test/modules
      ecto c++ test modules
  - test/python
      python unit tests of ecto. Try installing python-nose, and running nosetests from within this dir.
  
libraries
================================
  - libecto.so
      c++ library, clients must link against this and the 
  - ecto.so
      python bindings lib
  - buster.so
      a sample client python exposed module

Usage in client code
================================

Use should use cmake to find ecto and bring in a few macros:

.. code-block:: cmake

  find_package(ecto REQUIRED)
  
  #this takes care of linking against python and ecto
  ectomodule(buster
    test/modules/buster.cpp
  )
  
  ecto_link(buster
    ${MY_EXTRA_LIBS}
  )
