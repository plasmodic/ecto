module
======
The module is the basic unit of work for the ecto **DAG**.

anatomy of an an ecto module
----------------------------
The following is a sketch of a user ecto module.

.. code-block:: c++

  struct MyModule
  {
    //called first thing, the user should declare their parameters in this
    //free standing function.
    static void declare_params(tendrils& params);
    //declare inputs and outputs here. The parameters may be used to
    //determine the io
    static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
    //called right after allocation of the module, exactly once.
    void configure(tendrils& params, tendrils& in, tendrils& out);
    //called at every execution of the graph
    int process(const tendrils& in, tendrils& out);
    //called right before the destructor of the module, a good place to do
    //critical cleanup work.
    void destroy();
  };
  
A user's module need not implement all of these functions.
  
life cycle of an ecto module
----------------------------

The ecto module goes through a few states during its lifetime.

* Parameters declared
  This happens in a static function, before user modules are allocated.

  **This function must succeed without incident**
                                                		
* Inputs and Outputs declared 

  Given parameters that have been declared and optionally set by an
  external user, the module may declare inputs and outputs. Again this
  happens in a static function so that the module need not be
  allocated.

  **This function must succeed without incident**

* Module allocated and configuration

  Shortly after the module has been allocated, the configure function
  will be called. At this time the user should cache parameters or
  register parameter change callbacks. This will only be called once
  under normal circumstance.

* Processing

  The process call will be called at every execution of the graph.
	
* End of life 

  The module is about to be deallocated, and the destroy
  function is called by the system to allow any nasty cleanup that
  should occur, that may not take place in the module's destructor.
		
