.. _builtin_cells:

Builtin Ecto Cells
==================
Ecto contains a few cells that are generally useful for Ecto graph construction.

Convenience cells
-----------------
The are cells that may be generally useful in your ecto graphs and tend to be
weakly typed.

Constant
^^^^^^^^
This cell takes a python parameter that becomes a constant output. As long as
this python type is convertable to a c++ type(through python bindings or basic types),
it may be connected to cells written in c++.

.. ectocell:: ecto Constant

Passthrough
^^^^^^^^^^^
Passthrough is mostly useful from within a :ref:`black_box` to connect
multiple inputs to one cell, which then becomes a singular input for the BlackBox.

.. ectocell:: ecto Passthrough

Conditional Execution
---------------------
The If cell enables you to conditionally execute a single cell
based on an input flag.

.. ectocell:: ecto If

An example of using the If cell for condition execution of a
cell. Notice that cell that is encased in an If is **not added to the
graph**, it is passed to the ``If`` cell as a **parameter**.  

.. literalinclude:: conditional.py

The above sample uses the built in cell:

.. ectocell:: ecto TrueEveryN

Entanglement
------------

To support feedback loops, asynchronous execution of graphs, and
conditional execution, ecto supplies a concept of entangled cells,
which allow communication of values without breaking the acyclic or
synchronous nature of graph execution.

.. autofunction:: ecto.EntangledPair

  EntangledPair is useful when you would like to feed the output of
  one cell into the input of another cell, without creating a cycle in
  the graph. Keep in mind that the first execution of the graph will
  result in the default value of the whatever the source is connected
  to being used.

Here is an example of using the EntangledPair for feedback in a graph.

.. literalinclude:: feedback.py

