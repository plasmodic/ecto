Layout
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
---------------------------------------
  - libecto.so
      c++ library, clients must link against this and the 
  - ecto.so
      python bindings lib
  - buster.so
      a sample client python exposed module