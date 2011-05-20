Usage in client code
================================

Use should use cmake to find ecto and bring in a few macros:

.. code-block:: cmake

  find_package(ecto REQUIRED)
  
  #this takes care of linking against python and ecto
  ectomodule(buster
    test/modules/buster.cpp
  )
  
  ecto_link(buster
    ${MY_EXTRA_LIBS}
  )