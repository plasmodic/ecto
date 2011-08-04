.. _cells-intro:

Cells
+++++
The Cell is the core concept that one must grok to use ecto.  Think of a cell
as a small self contained well formed unit of processing machinery. Each cell's 
job is to take some number of inputs, and transform them into 
some number of outputs. Of course, parameters may have some effect on this
transformation, and cells may each have their own state that they alone govern.

Let us examine a c++ construct, a common functor:

.. _cell-Functor:

    .. literalinclude:: functor.cpp
	   :language: cpp
	   :start-after: //start
	   :end-before: //end
    
    A c++ function object.

Functors, function objects, are a reasonable way of encapsulating self contained
functionality to operate on a set of data, and mesh well with the stl algorithms.
The operator() may be abused by templated code to give way to genericity.

The ecto Cell is similar, in that it gives ecto a common unit with which to work.
Here is the ecto equivalent of the above functor, slightly more verbose of course:

.. _cell-Printer01:
   
   .. literalinclude:: cell.cpp
	   :language: cpp
	   :start-after: //start
	   :end-before: //end

   A typical :ref:`ecto::cell`.



The verbosity is a feature of ecto, in that each cell exposes as much information
as it can to the :ref:`ectosphere`.  At runtime, ecto may ask the cell
for documentation, types, and semantics.  These features are what enable
the type-safety in a graph of cells, or auto-completion from an :ref:`ipython context <ipython>`.

.. _examination-cell:

Examination of the Cell
^^^^^^^^^^^^^^^^^^^^^^^

A Cell defines some set of parameters, inputs and outputs statically.  And each cell
created from this static definition holds its own state, and may operate on it's
parameters,inputs and outputs at specific moments throughout it's lifetime. Also
the outside world may examine and manipulate the cells :ref:`ecto::tendrils`.

Let us look at a graphical representation of the Cell written :ref:`above <cell-Printer01>`:

.. figure:: Printer01.png
   
   A graphical representation of a :ref:`ecto::cell`.  In ``green`` are the inputs,
   ``blue`` parameters, ``yellow`` cell type. This cell has no outputs.

The parameters, inputs, and outputs of a cell all share the same type, :ref:`ecto::tendrils`.
Tendrils are mappings between strings, and lazily typed objects.  These are how the
cell communicates with the rest of the system.  The reason for choosing a runtime
typed object like the :ref:`ecto::tendril` instead of a compile time typed object
like a ``boost::tuple`` is that it allows for ecto to be type ignorant, as the
data held by the tendril has really no effect on how a graph executes or the python
interfaces.  Ecto is a plugin based architecture, and so can not be header only,
strictly compile time typed.


