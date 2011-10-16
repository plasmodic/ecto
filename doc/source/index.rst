Ecto
====

Ecto is a hybrid C++/Python development framework for constructing
efficient processing pipelines.  In Ecto, pipelines are defined in
terms of processing units, ``Cells``, connected by data paths,
``Tendrils``, that form *Directed Acyclic Graphs*, ``Plasms``. Cells
are typically written in C++, tendrils may be any copyable type, and
the flow of data through plasms is controlled by various external
schedulers.  Graphs are constructed with a small domain-specific
language hosted in Python.

Ecto may be useful in problem domains such as perception, audio, or
robotics.

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
   overview/index.rst
   using_ecto/index.rst
   tutorial/index.rst
   techniques/index.rst
   reference/index.rst
   cmake.rst
   release_checklist.rst
   dependencies.rst
   changelog.rst
   codingstandards.rst
   glossary.rst
