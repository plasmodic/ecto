cell
======
The cell is the basic unit of work for the ecto **DAG**.

anatomy of an an ecto cell
----------------------------
The following is a sketch of a user ecto cell.

.. code-block:: c++

  struct MyEctoCell
  {
    static void declare_params(tendrils& params);
    static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
    void configure(tendrils& params, tendrils& in, tendrils& out);
    int process(const tendrils& in, tendrils& out);
    void destroy();
  };
  
The purpose of this interface is to expose as much information about what the cell does to ecto, while providing
the implementer with a graceful degradation of functionality and form.  An ``ecto::cell`` can be thought of
as having any number of inputs, outputs, and parameters, and each instance has a state, which is retained over each
instance's lifetime.
  
life cycle of an ecto cell
----------------------------

The ecto cell goes through a few states during its lifetime.

* Parameters declared
  This happens in a static function, before user cells are allocated.

  **This function must succeed without incident**
                                                		
* Inputs and Outputs declared 

  Given parameters that have been declared and optionally set by an
  external user, the cell may declare inputs and outputs. Again this
  happens in a static function so that the cell need not be
  allocated.

  **This function must succeed without incident**

* Cell allocated and configurated

  Shortly after the cell has been allocated, the configure function
  will be called. At this time the user should cache parameters or
  register parameter change callbacks. This will only be called once
  under normal circumstance.

* Processing

  The process call will be called at every execution of the graph.
	
* End of life 

  The cell is about to be deallocated, and the destroy
  function is called by the system to allow any nasty cleanup that
  should occur, that may not take place in the cell's destructor.
		

Inherently thread-unsafe cells
------------------------------

Some cell types may be inherently thread-unsafe (e.g. they access
global/static state without locking); and therefore no two cell
instances of this same type should ever run concurrently.  See
:cmacro:`ECTO_THREAD_UNSAFE`.


