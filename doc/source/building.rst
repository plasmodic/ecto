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


Install
---------------------------------------

You may install ecto using the following:

.. code-block:: sh
  
  cd build
  sudo make install
  sudo ldconfig
  

This will install ecto to the appropriate system paths. On ubuntu the install may touch the following folders:

.. code-block:: sh

  /usr/local/include/ecto/
  /usr/local/share/ecto/
  /usr/local/lib/python2.6/dist-packages/
  

The advantage to installing ecto is that it becomes much easier for client code to use.  cmake will auto-magically 
be able to find ecto, and it will be in your pythonpath by default.

