.. _checlist:

Quickguide
==========

You definitely need to go through more docs but this is the rough outline of what happens when you write ecto code: you define
Python/C++ cells that will belong to a Python module. So you need to go through the following steps:

    * you need to define your ecto module through an ``ectomodule`` CMake macro:

      .. code-block:: cmake

          ectomodule(my_ecto_module_name INSTALL
                                         DESTINATION ./here/different_name
                                         module.cpp
                                         awesome_file1.cpp
                                         awesome_file2.cpp
                     )
    * You need a module.cpp file that simply defines the Python module containing your cells

      .. code-block:: c++

          #include <ecto/ecto.hpp>
          ECTO_DEFINE_MODULE(my_module) { }

    * each cell needs to define 4 C++ functions

      .. code-block:: c++

          static void
          declare_params(tendrils&)
          static void
          declare_io(const tendrils&, tendrils&, tendrils&)
          void
          configure(const tendrils&, const tendrils&, const tendrils&)
          int
          process(const tendrils&, const tendrils&)

    * you create a Python script to create your plasm and run it

      .. code-block:: python

          #!/usr/bin/env python
          import ecto

          plasm = ecto.Plasm()
          plasm.connect(cell1['output'] >> cell2['input'])

          sched = ecto.Scheduler(plasm)
          sched.execute(niter=2)

Now, let's see some concrete examples.
