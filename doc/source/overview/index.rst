.. _overview-ecto:

Overview of Ecto
================

:ref:`ecto-greek` is a scriptable processing framework with a
lightweight plugin architecture, graph constructs and scheduling
algorithms suitable for computer vision, perception,
audio processing and robotics pipelines. The key components of Ecto
are
:ref:`cells-overview`,
:ref:`plasms-overview`,
:ref:`tendrils-overview`,
and
:ref:`schedulers-overview`.
With these tools, you, the programmer, researcher or hacker may
construct well partitioned reusable processing graphs that exhibit qualities such
as ordered synchronous execution, efficient scheduling routines, type safety, and
self documentation.

To develop ecto based applications, one should be moderately comfortable with
C++ and Python.  C++ is preferred when writing ``Cells``, the basic unit of work in Ecto.
Python becomes the glue that is used to compose higher level processing
graphs, or ``Plasms`` from these cells.  As such, most of the examples that will
follow will involve both C++ and Python snippets.

.. toctree::
    :maxdepth: 1

    cells.rst
    modules.rst
