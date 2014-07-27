0.6.4 (2014-07-27)
------------------
* Posix signal handling
* proper depends syntax for pythonlibs find-packages variables.
* add the impl sub-package to the python setup.
* subprocessing view_plasm
* sigint handling for sync and async executors
* remove usage of SYSTEM in include_directories.
* experimental signal connection for sync executions.
* internal 0.4 xdot is broken on trusty, this moves it out to 0.5 as a rosdep entity.
* Contributors: Daniel Stonier, Vincent Rabaud

0.6.3 (2014-04-03)
------------------
* Merge pull request `#247 <https://github.com/plasmodic/ecto/issues/247>`_ from cottsay/master
  Added depend on python
* Added depend on python
* Contributors: Scott K Logan, Vincent Rabaud

0.6.2 (2014-03-02)
------------------
* get the tests to compile on OSX
* solve boost::bind problem on some compilers
* fixes `#245 <https://github.com/plasmodic/ecto/issues/245>`_ according to http://bugs.python.org/issue10910
* trust catkin to handle the version number
* trust catkin to handle ecto_LIBRARIES
* Contributors: Vincent Rabaud

0.6.1 (2014-02-16)
------------------
* get tests to pass with boost 1.54
* update maintainers
* fix compilation on Saucy
* fix warnings in the doc
* Contributors: Vincent Rabaud

0.6.0 (2014-01-26  15:37:06 +0100)
----------------------------------
- drop Fuerte support
- fix compilation errors on recent boost
