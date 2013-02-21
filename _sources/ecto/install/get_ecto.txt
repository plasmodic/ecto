Build Ecto From Source
======================

ecto is available here: https://github.com/plasmodic/ecto

It is also dependent on ``catkin``

.. code-block:: bash

  mkdir ecto_kitchen && cd ecto_kitchen
  git clone http://github.com/ros/catkin.git
  git clone http://github.com/ros-infrastructure/catkin_pkg.git
  git clone http://github.com/plasmodic/ecto.git
  ln -s catkin/cmake/toplevel.cmake CMakeLists.txt

You should see the following outputish:

::

    $ git clone git://github.com/plasmodic/ecto.git
    Initialized empty Git repository in /tmp/scratchy/ecto/.git/
    remote: Counting objects: 5011, done.
    remote: Compressing objects: 100% (1767/1767), done.
    remote: Total 5011 (delta 3226), reused 4711 (delta 2927)
    Receiving objects: 100% (5011/5011), 1.07 MiB | 994 KiB/s, done.
    Resolving deltas: 100% (3226/3226), done.


building ecto
------------------------------------

Using a standard cmake build system, you must first create a build directory and
run cmake to configure the build system. `cmake` may be run with all default settings for all but
the most advanced user.

.. code-block:: bash

  export PYTHONPATH=`pwd`/catkin_pkg/src:$PYTHONPATH
  mkdir build && cd build && cmake ..


You should see the following outputish:

::

    $ mkdir build
    $ cd build
    $ cmake ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /usr/bin/gcc
    ... etc etc
    -- Building ecto version: 0.1.0
    -- Building ecto code name: mold spore (alpha)
    -- flags:  -Werror -Wall -Wl,--no-undefined -O3 -DNDEBUG
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /tmp/scratchy/ecto/build


If that completed with out error or warnings you are ready to build ecto:

.. code-block:: sh

    make

You should see the following outputish:

::

    $ make
    Scanning dependencies of target ecto
    [  2%] Building CXX object src/lib/CMakeFiles/ecto.dir/abi.cpp.o
    [  4%] Building CXX object src/lib/CMakeFiles/ecto.dir/tendril.cpp.o
    ... etc etc
    Linking CXX shared library ../../lib/ecto_test.so
    [100%] Built target ecto_test_ectomodule


Now you have a working build of ecto! You should try to run a test.

.. code-block:: sh

    #cd to the root of the ecto kitchen
    cd ..
    #add ecto to your python path
    . build/devel/setup.bash
    python ecto/samples/hello.py

You should see the following outputish:

::

    digraph G {
    graph [rankdir=TB, ranksep=1]
    edge [labelfontsize=8]
    0[label="hello_ecto::Reader"];
    1[label="hello_ecto::Printer"];
    2[label="hello_ecto::Printer"];
    0->1 [headlabel="str" taillabel="output"];
    0->2 [headlabel="str" taillabel="output"];
    }

    Enter input, q to quit
    hello there ecto q
    hello
    hello
    there
    there
    ecto
    ecto
    q
    q

Dependencies
----------------------------------------

On ubuntu its simple....

.. code-block:: sh

    sudo apt-get install libboost-python-dev libboost-filesystem-dev libboost-system-dev \
            libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen \
            python-sphinx

Install
---------------------------------------

You may install ecto using the following:

.. code-block:: sh

  cd build
  sudo make install
  sudo ldconfig


This will install ecto to the appropriate system paths. On ubuntu the install may touch the following folders:

.. code-block:: sh

  /usr/local/include/ecto-VERSION/
  /usr/local/share/ecto-VERSION/
  /usr/local/lib/python*/dist-packages/


The advantage to installing ecto is that it becomes much easier for client code to use.  cmake will auto-magically
be able to find ecto, and it will be in your pythonpath by default.

Docs
------------------------------------------------

Docs may be generated from the source in the following manner.

.. code-block:: sh

	cd build
	make doc #for all documentaition
	make html #for sphinx (prefer this for usage docs)
	make pdf #sphinx pdf manual
	make doxygen #for c++ api docs
	ccmake . #edit doc options.

Tests
--------------------------------------------------

.. code-block:: sh

	cd build
	make test

or

.. code-block:: sh

	cd build
	ctest -V

