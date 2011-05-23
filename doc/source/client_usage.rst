Usage in client code
================================

Use should use cmake to find ecto and bring in a few macros:

.. code-block:: cmake

  cmake_minimum_required(VERSION 2.8)
  project(ecto_samples)
  
  find_package(ecto REQUIRED)
  
  ectomodule(hello_ecto
      hello_ecto.cpp
  )
  
  #optionally link against other libs
  #ecto_link(hello_ecto
  #  ${MY_EXTRA_LIBS}
  #)

.. code-block:: sh
  
  mkdir build
  cd build
  cmake ..
  make
  

If you installed ecto, this will most likely just work.
You may then setup your python path, by sourcing the generated python_path.sh

.. code-block:: sh
  
  . build/python_path.sh

If you have a look at the file this is all it does:

.. code-block:: sh

  export PYTHONPATH=~/ecto_samples/build:/usr/local/lib/python2.6/dist-packages:$PYTHONPATH
  

