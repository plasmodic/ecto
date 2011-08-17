
Boost
=====

(Accurate as of boost 1.47.0)

If you want to build against latest boost (i.e. boost that isn't
available as a package) on linux, you need to use specific build
flags.  So that boost's obnoxious build system generates files that
cmake's ``find_package()`` can find.  First you run the
``bootstrap.sh``, then run bjam like this::

  ./bjam threading=multi link=shared runtime-link=shared --layout=tagged install

If you're on ubuntu I recommend using `checkinstall
<https://help.ubuntu.com/community/CheckInstall>`_ for easy removal of
the package if you get things wrong.

  

