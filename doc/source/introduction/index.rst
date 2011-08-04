.. toctree::
   :maxdepth: 1

.. _introduction-ecto
Introduction
============

:ref:`ecto-greek` is a scriptable processing framework with a
lightweight plugin architecture, graph constructs and scheduling
algorithms suitable for computer vision, perception,
audio processing and robotics pipelines. The key components of Ecto 
are :ref:`cells-intro`,
:ref:`plasms-intro`, :ref:`tendrils-intro`, 
and :ref:`schedulers-intro`.
With these tools, you, the programmer, researcher or hacker may productively
construct well partitioned reusable processing graphs that exhibit qualities such
as ordered synchronous execution, efficient scheduling routines, type safety, and
self documentation.

To develop ecto based applications, one should be moderately comfortable with
C++ and Python.  C++ is preferred when writing the ``Cell``, or the basic
unit of work in Ecto. Python is the glue that is used to compose higher level
graphs, or ``Plasms`` from these cells.  As such, most of the examples that will
follow will involve both C++ and Python snippets.

.. _cells-intro:

Cells
+++++
The Cell is the core concept that one must grok to use ecto.  Think of a cell
as a small self contained well formed unit of processing machinery. Each cell's 
job is to take some number of inputs, and transform them into 
some number of outputs. Of course, parameters may have some effect on this
transformation, and cells may each have their own state that they allow govern.

Let us examine a c++ construct, a common functor

.. literalinclude:: functor.cpp
   :language: cpp
   :start-after: //start
   :end-before: //end
   
.. _plasms-intro:

Plasms
++++++

.. _tendrils-intro:

Tendrils
++++++++

.. _schedulers-intro:

Schedulers
++++++++++