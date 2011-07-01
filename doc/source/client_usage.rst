Usage in client code
================================

Clients should use cmake to find ecto and bring in a few macros:

.. code-block:: cmake

    cmake_minimum_required(VERSION 2.8)
    project(ecto_samples)
    
    find_package(ecto REQUIRED)
    
    ectomodule(tutorial
        tutorial.cpp
    )
    
    #optionally link against other libs
    #ecto_link(hello_ecto
    #  ${MY_EXTRA_LIBS}
    #)
    
    #generates an install target for your module (puts it in the python dist folder)
    install_ecto_module(tutorial)
    
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
You may find sample of ecto usage in the samples directory.

An Example
-----------------------------------

CMakeLists.txt
***********************************

Here is the clients CMakeLists.txt

.. code-block:: cmake
    
    cmake_minimum_required(VERSION 2.8)
    project(ecto_samples)
    
    find_package(ecto REQUIRED)
    
    ectomodule(tutorial
        tutorial.cpp
    )
    
    #optionally link against other libs
    #ecto_link(hello_ecto
    #  ${MY_EXTRA_LIBS}
    #)
    
    #generates an install target for your module (puts it in the python dist folder)
    install_ecto_module(tutorial)
  
C++ Code
***********************************

An example implementation of an ecto module:

.. code-block:: c++
  
	#include <ecto/ecto.hpp>
	#include <iostream>
	
	namespace tutorial
	{
	  using ecto::tendrils;
	
	  struct Printer01
	  {
	    int
	    process(const tendrils& /*in*/, tendrils& /*out*/)
	    {
	      std::cout << "Hello" << std::endl;
	      return ecto::OK;
	    }
	  };
	
	}
	//declare the python module as tutorial, must be same name as the ectomodule target in CMakeLists.txt
	ECTO_DEFINE_MODULE(tutorial){}
	//register our cell with the existing ecto module 'tutorial' that is declared in tutorial.cpp
	ECTO_MODULE(tutorial, tutorial::Printer01, "Printer01", "Prints 'Hello' to standard output.");

Python code
*************************************************
Here is a snippet of python code that uses the modules above.

.. code-block:: python

	import ecto #this must be imported before other ecto based python modules
	import tutorial #this is our ecto module
	
	#allocate a Printer01 (defined in t0.cpp)
	printer = tutorial.Printer01()
	
	#Create a Plasm, the graph structure of ecto
	plasm = ecto.Plasm()
	
	#insert our instance into the graph so that it may be executed
	plasm.insert(printer)
	
	#execute in a tight loop 10 times
	plasm.execute(10)
	