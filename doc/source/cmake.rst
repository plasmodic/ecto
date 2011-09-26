The build system
================

Variables
---------

.. rubric:: KITCHEN_PROJECTS

Override this on the commandline to specify that only a subset of
projects should be built::

  cmake ../src '-DKITCHEN_PROJECTS=ecto;openni'

Don't forget that you have to quote the entire argument and separate
entries with semicolons, as above.

