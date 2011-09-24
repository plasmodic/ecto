ecto
====

Ecto is a hybrid C++/Python development framework for constructing and maintaining
pipelines.  In Ecto, pipelines are constructed in terms of processing units, ``Cells``,
connected by data paths, ``Tendrils``, that form *Directed Acyclic Graphs*, ``Plasms``.
Cells are typically written in C++, tendrils may be any type, and the plasm may
be executed in a variety of clever ways. Python is uses as a the graph DSL.

Ecto may be found useful in domains such as perception, audio, or robotics.

To get started see the online docs at http://ecto.willowgarage.com
