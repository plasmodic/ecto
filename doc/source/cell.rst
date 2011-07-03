cell
======
The cell is the basic unit of work for the ecto **DAG**.

anatomy of an an ecto cell
----------------------------
The following is a sketch of a user ecto cell.

.. code-block:: c++

  struct MyEctoCell
  {
    //called first thing, the user should declare their parameters in this
    //free standing function.
    static void declare_params(tendrils& params);
    //declare inputs and outputs here. The parameters may be used to
    //determine the io
    static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
    //called right after allocation of the cell, exactly once.
    void configure(tendrils& params, tendrils& in, tendrils& out);
    //called at every execution of the graph
    int process(const tendrils& in, tendrils& out);
    //called right before the destructor of the cell, a good place to do
    //critical cleanup work.
    void destroy();
  };
  
A user's cell need not implement all of these functions.
  
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
		
