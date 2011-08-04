.. index:: valgrind, gdb

Debugging things
================

Running an ecto script under gdb
--------------------------------

You'll notice that you can't run a vanilla script under gdb::

  % gdb ../src/ecto/test/scripts/test_random.py                       
  GNU gdb (Ubuntu/Linaro 7.2-1ubuntu11) 7.2

     [ blabber ]

  "/ssd/ecto_kitchen/src/ecto/test/scripts/test_random.py": not in executable format: File format not recognized

The trick is to run ``gdb`` on the python binary, not the script, and
pass the script as an argument.  ``gdb`` will detect when the script
imports modules of ecto cells and debug those too.  Use gdb's
``--args`` argument, for instance::

  % gdb --args /usr/bin/python ../ecto/test/scripts/test_random.py
  GNU gdb (Ubuntu/Linaro 7.2-1ubuntu11) 7.2
  Copyright (C) 2010 Free Software Foundation, Inc.
  License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
  This is free software: you are free to change and redistribute it.
  There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
  and "show warranty" for details.
  This GDB was configured as "x86_64-linux-gnu".
  For bug reporting instructions, please see:
  <http://www.gnu.org/software/gdb/bugs/>...
  Reading symbols from /usr/bin/python...(no debugging symbols found)...done.
  (gdb) 

At this point, since you haven't run the script yet, gdb won't know
about the cells inside any of the compiled modules that the script
imports.  You can load them manually::

  (gdb) symbol-file lib/ecto_test.so
  Reading symbols from /ssd/ecto_kitchen/build/lib/ecto_test.so...(no debugging symbols found)...done.

Notice above that there are no debug symbols in the library.  This
means my ``CMAKE_BUILD_TYPE`` was not set to ``Debug`` or
``RelWithDebInfo``, and this is a problem.  Once I've rebuilt,

::




