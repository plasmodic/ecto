
.. highlight:: ectosh

.. _client-usage:

Using ecto in your own projects
===============================


CMake is the recommended build system tool.  For the purpose of this
documentation you should have a simple CMake project that looks like this on disk.

::

  % ls my_ecto_project
  CMakeLists.txt
  hello_ecto.cpp
  hello.py


CMake Setup
-----------

CMake should be used to find ecto and bring in a few macros:

.. code-block:: cmake

    cmake_minimum_required(VERSION 2.8)

    project(my_ecto_project)

    #defines include directories, libraries a few macros
    #that simplify creating ecto module targets.
    find_package(ecto REQUIRED)

    #make all libraries appear in the build/lib directory
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

    ectomodule(hello_ecto ecto FALSE
        hello_ecto.cpp
    )

    #optionally link against other libs
    #ecto_link(hello_ecto
    #  ${MY_EXTRA_LIBS}
    #)

Take notice of the command ``find_package(ecto REQUIRED)``. If ecto is installed on your
system, or built somewhere, this will enable your project to become an ecto beast.

ecto is installed
^^^^^^^^^^^^^^^^^

CMake should complete without error, using default settings.

::

    % cd my_ecto_project
    % mkdir build
    % cd build
    % cmake ..

Make sure that the output does not contain warnings or errors:

::

    % cmake ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /usr/bin/gcc
    -- Check for working C compiler: /usr/bin/gcc -- works
    ... etc etc
    -- Configuring done
    -- Generating done

ecto is not installed
^^^^^^^^^^^^^^^^^^^^^

You will need to supply the ecto build path to your cmake cache.

::

    % cd my_ecto_project
    % mkdir build
    % cd build
    % cmake -Decto_DIR=~/ecto_dev/ecto/build ..

Make sure that the output does not contain warnings or errors:

::

    % cmake ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /usr/bin/gcc
    -- Check for working C compiler: /usr/bin/gcc -- works
    ... etc etc
    -- Configuring done
    -- Generating done

Build it.
^^^^^^^^^

::

    % cd my_ecto_project/build
    % make

