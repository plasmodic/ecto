Building
================================
Using a standard cmake build system:

::
  
  mkdir build
  cd build
  cmake ..
  make
  

The libs will be located in the build folder under lib

Assuming you're in the top level ecto directory, try the following in a shell:

::

  #add ecto to your python path
  . build/python_path.sh
  python test/python/test_plasm.py

Dependencies
----------------------------------------

::

	 sudo apt-get install libboost-python-dev libboost-filesystem-dev libboost-system-dev libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen
	 sudo easy_install sphinx

Install
---------------------------------------

You may install ecto using the following:

::

  cd build
  sudo make install
  sudo ldconfig
  

This will install ecto to the appropriate system paths. On ubuntu the install may touch the following folders:

::

  /usr/local/include/ecto/
  /usr/local/share/ecto/
  /usr/local/lib/python2.6/dist-packages/
  

The advantage to installing ecto is that it becomes much easier for client code to use.  cmake will auto-magically 
be able to find ecto, and it will be in your pythonpath by default.

Docs
------------------------------------------------
::

	cd build
	make doc #for doxygen
	make sphinx_doc #for sphinx (prefer this for usage docs)

Tests
--------------------------------------------------
::

	cd build
	make test

or

::

	cd build
	ctest -V

