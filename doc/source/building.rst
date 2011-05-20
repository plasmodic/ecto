
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
