.. _tutorial-hello-plasms:

Hello Plasms!
=============

Ok, so you made your first cell in :ref:`tutorial-hello-ecto`.  Now lets make
a graph, what ecto is especially good at.

c++
---

Lets make two more cells

Download: :download:`srcs/Increment.cpp`

.. literalinclude:: srcs/Increment.cpp
    :language: cpp

Download: :download:`srcs/Add.cpp`

.. literalinclude:: srcs/Add.cpp
    :language: cpp

python
------
The python counter part to hello might look like:

Download: :download:`srcs/plasms.py`

.. literalinclude:: srcs/plasms.py
  :language: python


Lets run it:

.. program-output:: srcs/plasms_doc.py
   :in_srcdir:

Parameters
^^^^^^^^^^
Look at the allocation of our cells in the script.

.. literalinclude:: srcs/plasms.py
  :language: py
  :lines: 5-8

The parameters that were declared in:

.. literalinclude:: srcs/Add.cpp
    :language: cpp
    :lines: 8-14

Become the keyword arguments for the cell.  The first non keyword argument is
the cell's instance name, which is relevant for pretty looking graphs.

Connections
^^^^^^^^^^^^
The syntax for connecting ecto cells together in the
plasm looks like this.

.. literalinclude:: srcs/plasms.py
  :language: py
  :lines: 10-17

Each connection is made by using the ``>>`` operator, where
the key value in the ``[]`` operator refers to an output tendril
on the left hand side, or an input tendril on the right hand side.
See :ref:`tendril-connections` for more details on how this operator
works.


Graphs
^^^^^^

The graph for this will look like:

.. ectoplot:: srcs/plasms_doc.py plasm

Plasms, graphs, in ecto are *Directed Acyclic Graphs* , see :ref:`DAG`.
The DAG is important for scheduling and determining execution order.
It is used to describe the **happens before** relationships in cells.
Because of the DAG construct, implicit feedback loops are not allowed,
and can only be achieved through special purpose cells,
see :ref:`entanglement` for more information.

Execution
^^^^^^^^^

Once a plasm is constructed, it may be used with an :ref:`ecto scheduler <schedulers>`

.. literalinclude:: srcs/plasms.py
  :language: py
  :lines: 18-22

..  _topological-sort: http://en.wikipedia.org/wiki/Topological_sorting

The :py:class:`ecto.Scheduler` scheduler essentially
does a `topological-sort`_
on the graph, and then executes each cell in order.
This repeats for the specified number of iterations,
or indefinitely if ``niter`` is ``0``.

