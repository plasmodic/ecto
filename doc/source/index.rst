ecto
====

ecto is a dynamically configurable *Directed Acyclic processing Graph* **(DAG)** framework.  
Users may write reusable ``ecto::module``\ s which 
become the nodes in the DAG, or ``ecto::plasm``.  Modules may be written 
in c++ as ``boost::python`` extensions, or in pure python, 
and python is used to construct the DAG.

Contents:
=========

.. toctree::
   :maxdepth: 2
   
   building
   tutorial
   autoregister
   client_usage
   auto_docs
   tendril
   tendrils
   cell
   plasm
   schedulers
   autoregister
   
ecto at a glance
----------------

    * Simple processing node interface for building your own modules.
    
    .. code-block:: c++
    
        #include <ecto/ecto.hpp>
        using ecto::tendrils;
        struct MyModule
        {
          static void declare_params(tendrils& params);
          static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
          void configure(tendrils& params, tendrils& in, tendrils& out);
          int process(tendrils& in, tendrils& out);
          void destroy();
        };

    
    * Inputs, outputs and parameters are templated, and type erasing, giving typesafety and the ability to use your
      own data types..
      
    .. code-block:: c++
        
        void MyModule::declare_params(tendrils& params)
        {
            params.declare<Foo>("foo","Foo is for spam. This is a doc string", 
            					Foo(3.14));
            params.declare<std::string>("str", "str is a standard string.", "default");
        }
    
    * Python is used as the plugin architecture of ecto. Exposing your modules to python is dead simple.
      The use of boost::python means that the python bindings for your data types are an optional powerful tool.
      
    .. code-block:: c++
        
        BOOST_PYTHON_MODULE(hello_ecto)
        {
          ecto::wrap<Printer>("Printer", "Prints a string input to standard output.");
          ecto::wrap<Reader>("Reader", "Reads input from standard input.");
        }
        
    * ecto forces your modules to be somewhat self documenting, and allows full
      introspection from python and c++, including
      type names, docstrings and variable names.
    * The plasm (DAG) executes in compiled code.
    * Python is used for declaring the processing graph, or as its known to ecto, the *plasm*.
    
	
	.. code-block:: python
    
		import ecto #ecto core library
		import hello_ecto #a user library, that has a few ecto modules
		
		#instantiate a plasm, our DAG structure
		plasm = ecto.Plasm()
		
		#instantiate processing modules
		r = hello_ecto.Reader()
		
		#notice the keyword args, these get mapped
		#as parameters
		p = hello_ecto.Printer(str="default")
		
		#connect outputs to inputs
		plasm.connect(r["output] >> p["str"])
		
		#an execution loop
		print "Enter input, q to quit"
		while r.outputs.output != 'q':
		  plasm.execute() #this executes the graph  

                
    * The ecto::plasm is easily inspected using graphviz tools.
    
    .. image:: images/plasm.png
    
    .. code-block :: c++
    
		digraph G {
			graph [rankdir=TB, ranksep=1]
			edge [labelfontsize=8]
			0[label="hello_ecto::Reader"];
			1[label="hello_ecto::Printer"];
			2[label="hello_ecto::Printer"];
			0->1 [headlabel="str" taillabel="output"];
			0->2 [headlabel="str" taillabel="output"];
		}

    * Each module is self documenting by design.
