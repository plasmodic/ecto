Ecto
====

Ecto is a hybrid C++/Python development framework for constructing efficient
processing pipelines.  In Ecto, pipelines are defined in terms of processing
units, ``Cells``, connected by data paths, ``Tendrils``,
that form *Directed Acyclic Graphs*, ``Plasms``. Cells are typically written
in C++, tendrils may be any type, and plasms may be scheduled in a variety of
clever ways. Python is uses as the graph DSL.

Ecto may be found useful in domains such as perception, audio, or robotics.

.. rubric:: IRC

Drop in on us on ``#ecto`` on ``irc.oftc.net``

.. rubric:: Email List

Also feel free to join the email list:

* site: http://groups.google.com/a/plasmodic.org/group/ecto-dev
* email: **ecto-dev@plasmodic.org**

.. rubric:: Table of Contents

.. toctree::
   :maxdepth: 1

   motivation.rst
   changelog.rst
   overview/index.rst
   using_ecto/index.rst
   tutorial/index.rst
   examples/index.rst
   techniques/index.rst
   reference/index.rst
   cmake.rst
   release_checklist.rst
