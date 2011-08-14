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

.. program-output:: srcs/plasms.py
   :in_srcdir:

Connections
^^^^^^^^^^^^
The syntax for connecting ecto cells together in the
plasm looks like this.

.. literalinclude:: srcs/plasms.py
  :language: py
  :lines: 10,11,12

Each connection is made by using the ``>>`` operator, where
the key value in the ``[]`` operator refers to an output tendril
on the left hand side, or an input tendril on the right hand side.
See :ref:`tendril-connections` for more details on how this operator
works.

The graph for this will look like:

  .. ectoplot:: srcs/plasms.py plasm

  