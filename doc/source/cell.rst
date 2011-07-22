cell
====

The cell is the basic unit of work for the ecto :ref:`DAG`.


Anatomy of an an ecto cell
--------------------------
The following is a sketch of a user ecto cell.

.. code-block:: c++

  struct MyCell
  {
    static void declare_params(tendrils& params);
    static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
    void configure(tendrils& params, tendrils& in, tendrils& out);
    int process(tendrils& in, tendrils& out);
    void destroy();
  };
  
The purpose of this interface is to expose as much information about
what the cell does to ecto, while providing the implementer with a
graceful degradation of functionality and form.  An ``ecto::cell`` can
be thought of as having any number of inputs, outputs, and parameters.
  
Life cycle of an ecto cell
--------------------------

The ecto cell goes through a few states during its lifetime.  

* ``MyCell::declare_params()`` is called once per cell (maybe more) *before*
  user cells are allocated; herein the user communicates the cell's
  parameters, doc strings, default values and constraints to the
  system.  It must succeed without incident.
                                                		
* ``MyCell::declare_io()`` is called once per cell (maybe more), before
  allocation, to describe to the system the inputs and outputs of the
  cell, given the set of parameters supplied.  This function also must
  succeed without incident.

* ``MyCell::MyCell()``.  The cell will be default constructed.  There
  is currently no facility for providing arguments to a cell's
  constructor.

* ``MyCell::configure()`` will be called after the cell has been
  allocated. At this time the user should cache parameters or register
  parameter change callbacks. This will only be called once.

* ``MyCell::process()`` called some number of times.  If the
  user-supplied values for parameters change, any registered parameter
  change callbacks will be called first.
	
* ``MyCell::destroy()`` will be called once, after the graph is done
  executing, before deallocation.


Inherently thread-unsafe cells
------------------------------

Some cell types may be inherently thread-unsafe (e.g. they access
global/static state without locking); and therefore no two cell
instances of this same type should ever run concurrently.  See
:cmacro:`ECTO_THREAD_UNSAFE`.


