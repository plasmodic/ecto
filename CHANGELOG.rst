0.6.7 (2014-11-11)
------------------
* Merge branch 'master' of github.com:plasmodic/ecto
* Merge pull request `#266 <https://github.com/plasmodic/ecto/issues/266>`_ from stonier/if_nests
  Nested If Cells
* test for nested ifs.
* parameterise if cell name variables, this allows if-nests.
* add awesome Daniel as maintainer
* Merge pull request `#265 <https://github.com/plasmodic/ecto/issues/265>`_ from stonier/gil_releaser
  Gil releaser
* Merge pull request `#264 <https://github.com/plasmodic/ecto/issues/264>`_ from stonier/directed_configuration
  Directed configuration revisited
* bugfix module reference in the gil exercise.
* advanced multithreaded gil releaser.
* directed configuration
* Contributors: Daniel Stonier, Vincent Rabaud

0.6.6 (2014-09-06)
------------------
* don't forget to install necessary extra CMake files
* Contributors: Vincent Rabaud

0.6.5 (2014-09-06)
------------------
* do not install dev ectoConfig*
  fixes `#262 <https://github.com/plasmodic/ecto/issues/262>`_
* Merge pull request `#261 <https://github.com/plasmodic/ecto/issues/261>`_ from stonier/bugfix_pythonic_cells
  Bugfix pythonic cell construction
* Merge pull request `#263 <https://github.com/plasmodic/ecto/issues/263>`_ from stonier/dot_graph_titles
  Add title to the dot graph windows
* add title to the dot graph windows.
* bugfix pythonic cell construction and parameter argument invocation.
* Merge pull request `#259 <https://github.com/plasmodic/ecto/issues/259>`_ from stonier/dynamic_reconfigure_whitelist
  A whitelist for dynamic reconfigure
* Merge pull request `#254 <https://github.com/plasmodic/ecto/issues/254>`_ from stonier/optional_inputs
  optional processing of connected inputs only
* Merge pull request `#258 <https://github.com/plasmodic/ecto/issues/258>`_ from stonier/dynamic_reconfigure
  Boolean types for dynamic reconfigure
* Use the value, don't assume it is true.
* a whitelist for dynamic reconfigure.
* bugfix boolean parameter setting in dynamic reconfigure.
* checkboxes for dynamic reconfigure.
* optional processing of connected inputs only.
* Contributors: Daniel Stonier, Vincent Rabaud

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
