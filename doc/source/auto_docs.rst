Automatic documentation of user modules
=======================================
The implicit hard coded docstrings in user's code, during parameter and input/output declaration
may be introspected from simple scripts, to give nice looking auto documentation.

Assuming you have installed ecto, and you would like to see the documentation for one of your modules
you may do the following. (Also assumes your module is in the python path.)

::

	ecto_doc.py mymodule > doc.rst
	restview doc.rst
	#or
	rst2pdf doc.rst
	evince doc.pdf

Tip: install restview for reading rst documentation in a web browser easily. Or install rst2pdf to
generate nice looking pdfs of your docs

::

	sudo easy_install restview
	sudo easy_install rst2pdf
	
Sample Generated Module Documentation
-------------------------------------
Since the modules are by design self documenting, it is simple to output rst for a given set of modules.

Printer (ecto::module)
-------------------------------------


Prints a string input to standard output.

params
****************************************

 - str [std::string] default = hello

    The default string to print

inputs
****************************************

 - str [std::string] default = hello

    The string to print.


Reader (ecto::module)
-------------------------------------

Reads input from standard input.

outputs
****************************************

 - output [std::string] default = 

    Output from standard in
