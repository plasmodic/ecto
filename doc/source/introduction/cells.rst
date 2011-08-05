.. _cells-intro:

Cells
=====
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
+++++++++++++++++++++++

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

Optional interface
------------------

The most basic of cells would be:

  .. literalinclude:: nop.cpp
    :language: cpp
    :start-after: //start
    :end-before: //end


Each cell may or may not implement the following functions:

.. _interface_sample:

  .. literalinclude:: interface.cpp
     :language: cpp
     :start-after: //start
     :end-before: //end
     
  The cell interface functions.
  
.. sidebar: A note on where to implement
  
  Notice that in most examples we implement the functions for cells inline in the declaration
  of the struct. However there is nothing stopping you from writing the implementation elsewhere:
  
    .. literalinclude:: interface.cpp
     :language: cpp
     :start-after: //impl_start
     :end-before: //impl_end

However if you do implement any of the methods in the cell interface, be sure that
their signatures match the above specification.


A peak under the covers
-----------------------

.. epigraph::
  
  But wait, how does ecto know anything about my cell? You must be full of black magic!
  Where is the inheritence and polymorphism?
  
  -- Egon

You might be thinking these kinds of things... Well the registration with ecto occurs in the macro :cmacro:`ECTO_CELL`.
This macro
does some amount of extra fanciness, but in the end it takes your struct and does something similar
to the following simplified example:

.. code-block:: c++
  
  struct cell
  {
    virtual ~cell(){}
    virtual void declare_params() = 0;
    virtual int process() = 0;
    //... other interface functions
    ecto::tendrils params_, inputs_, outputs_;
  };

  template<typename YourCell>
  struct cell_ : cell
  {
    void declare_params()
    {
      YourCell::declare_params(params_);
    }
    //... dispatch other functions...
    YourCell* thiz_;
  }
  
  cell_<InterfaceCell> c;
  
.. _SFINAE: http://en.wikipedia.org/wiki/Substitution_failure_is_not_an_error

The real implementation uses `SFINAE`_ to enable
optional implementation of the interface functions.
The macro :cmacro:`ECTO_CELL` also constructs python
bindings for your cell, and
generates RST formated doc strings from the
static declaration parameter and io functions.
The above sample
should be referred to as the essence of the technique, rather than the exact
implementation.  This technique gives a certain amount of opaqueness to
the client cell implementers, and
provides ecto with a flexible entry point for implementation details.

Doing work
----------

Let us take the :ref:`Printer <cell-printer01>` from above and use it from python. The following python
script demonstrates the python interface of our cell that ecto provides for free(assuming you used the macro and followed the cell interface).

.. literalinclude:: cell01.py
   :language: py

The script, when run will give the following output:

.. program-output:: doc/source/introduction/cell01.py
