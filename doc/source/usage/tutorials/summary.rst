.. _checlist:

What We've Learned So Far
=========================

After those tutorials, we are aware of the following:

    * ecto is very cool :)
    * you need to define your ecto module through an ``ectomodule`` CMake macro:
      .. code-block:: CMake
      
          ectomodule(my_ecto_module_name INSTALL
                                         DESTINATION ./here/different_name
                                         module.cpp
                                         awesome_file1.cpp
                                         awesome_file2.cpp
                     )
    * You need a module.cpp file that simply defines the Python module using
      .. code-block::c++

          #include <ecto/ecto.hpp>
          ECTO_DEFINE_MODULE(my_module) { }

    * each cell needs to define 4 functions
      .. code-block::c++

          static void
          declare_params(tendrils&)
          static void
          declare_io(const tendrils&, tendrils&, tendrils&)
          void
          configure(const tendrils&, const tendrils&, const tendrils&)
          int
          process(const tendrils&, const tendrils&)

    * you create a Python script to create your plasm and run it
      .. code-block::Python

          #!/usr/bin/env python
          import ecto

          plasm = ecto.Plasm()
          plasm.connect(cell1['output'] >> cell2['input'])

          sched = ecto.schedulers.Singlethreaded(plasm)
          sched.execute(niter=2)

Now, it's time to move to more advanced stuff.
