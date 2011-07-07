ecto
====

ecto is a dynamically configurable *Directed Acyclic processing Graph* **(DAG)** framework.  
Users may write reusable ``ecto::cell``\ s which 
become the nodes in the DAG, or ``ecto::plasm``.  Modules may be written 
in c++ as ``boost::python`` extensions, or in pure python, 
and python is used to construct the DAG.

.. toctree::
   :maxdepth: 2
   
   at_a_glance
   get_ecto
   autoregister
   client_usage
   auto_docs
   tendril
   tendrils
   cell
   plasm
   schedulers
