Python Binding Autoregistration
===============================

You may autoregister python bindings for modules.  This makes it
possible to store each module's definition in a different translation
unit, reducing recompile times.



.. c:macro:: ECTO_MODULE(pymodule_name, cell_type_name, "CellTypeName", "Cell Docstring")

Mark each ecto module as such with the ``ECTO_MODULE`` macro::

  struct Add {
    int process(const ecto::tendrils& inputs, ...)
    // ...
  };

  ECTO_MODULE(ecto_test, Add, "Add", "Add two doubles");

Where the arguments are 1. Python module name, but not as a
string, 2. Type of module, 3.  string name of module, 4. docstring for
module.

This may be placed in any translation unit within the shared library,
e.g. in a file Add.cpp that contains the code above.  

.. c:macro:: ECTO_DEFINE_MODULE(pymodule_name)

In a single place inside the shared library (e.g. ``module.cpp``, call
``ECTO_DEFINE_MODULE``::

  #include <ecto/ecto.hpp>
  #include <ecto/register.hpp>
  
  ECTO_DEFINE_MODULE(ecto_test)
  {
    // additional boost::python or ecto bindings go here
  }


