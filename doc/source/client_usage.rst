Usage in client code
================================

Use should use cmake to find ecto and bring in a few macros:

  .. code-block:: cmake

    cmake_minimum_required(VERSION 2.8)
    project(ecto_samples)
    
    find_package(ecto REQUIRED)
    
    ectomodule(hello_ecto
        hello_ecto.cpp
    )
    
    #optionally link against other libs
    #ecto_link(hello_ecto
    #  ${MY_EXTRA_LIBS}
    #)

  .. code-block:: sh
  
    mkdir build
    cd build
    cmake ..
    make
  

If you installed ecto, this will most likely just work.
You may then setup your python path, by sourcing the generated python_path.sh

.. code-block:: sh
  
  . build/python_path.sh

If you have a look at the file this is all it does:

.. code-block:: sh

  export PYTHONPATH=~/ecto_samples/build:/usr/local/lib/python2.6/dist-packages:$PYTHONPATH
  

Samples
-----------------------------------
You may want to checkout the samples, at https://github.com/ethanrublee/ecto_samples

An Example
-----------------------------------

CMakeLists.txt
***********************************

Here is the clients CMakeLists.txt

  .. code-block:: cmake
    
    cmake_minimum_required(VERSION 2.8)
    project(ecto_samples)
    
    find_package(ecto REQUIRED)
    
    ectomodule(hello_ecto
        hello_ecto.cpp
    )
    
    #optionally link against other libs
    #ecto_link(hello_ecto
    #  ${MY_EXTRA_LIBS}
    #)
    
  
  
C++ Code
***********************************

An example implementation of an ecto module:

  .. code-block:: c++

    #include <ecto/ecto.hpp>
    #include <iostream>
    
    namespace hello_ecto
    {
    
    using ecto::tendrils;
    
    struct Printer
    {
      static void declare_params(tendrils& params)
      {
        params.declare<std::string> ("str", "The default string to print", "hello");
      }
    
      static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
      {
        in.declare<std::string> ("str", "The string to print.", parms.get<std::string> ("str"));
      }
    
      void configure(tendrils& params, tendrils& in, tendrils& out)
      {
        str_ = params.get<std::string> ("str");
      }
    
      int process(const tendrils& in, tendrils& /*out*/)
      {
        std::cout << in.get<std::string> ("str") << std::endl;
        return 0;
      }
      std::string str_;
    };
    
    struct Reader
    {
      static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
      {
        out.declare<std::string> ("output", "Output from standard in");
      }
    
      int process(const tendrils& in, tendrils& out)
      {
        std::string o;
        std::cin >> o;
        out.get<std::string> ("output") = o;
        return 0;
      }
    };
    
    }
    
    BOOST_PYTHON_MODULE(hello_ecto)
    {
      using namespace hello_ecto;
      ecto::wrap<Printer>("Printer", "Prints a string input to standard output.");
      ecto::wrap<Reader>("Reader", "Reads input from standard input.");
    }

Python code
*************************************************
Here is a snippet of python code that uses the modules above.

.. code-block:: python

	#!/usr/bin/env python
	import ecto #ecto core library
	import hello_ecto #a user library, that has a few ecto modules
	
	debug = True
	
	def mygraph():
	    #instantiate a plasm, our DAG structure
	    plasm = ecto.Plasm()
	    
	    #allocate processing modules
	    r = hello_ecto.Reader()
	    
	    #notice the keyword args, these get mapped
	    #as parameters
	    p1 = hello_ecto.Printer(str="default")
	    p2 = hello_ecto.Printer(str="default")
	    
	    #connect outputs to inputs
	    plasm.connect(r, "output", p1, "str")
	    plasm.connect(r, "output", p2, "str")
	    
	    if debug:
	        #render the DAG with dot
	        print plasm.viz()
	        ecto.view_plasm(plasm)
	    
	    #an execution loop
	    print "Enter input, q to quit"
	    while r.outputs.output != 'q':
	        plasm.execute() #this executes the graph in compiled code.
	
	if __name__ == '__main__':
	    mygraph()


