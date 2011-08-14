.. _tutorial-hello-ecto:

Hello Ecto!
===========

The beginning. Well, you should have read :ref:`client-usage`
first. Your ecto experience will involve a hybrid approach to development, using
c++ and python. 

c++
---

So, the c++ side of things.  Ecto code structurally break down into 
:ref:`Cells <cells-overview>` and :ref:`Modules <modules-overview>`

The module code:

  .. _code-module:
 
  .. literalinclude:: srcs/tutorial.cpp
     :language: cpp
  
The cell code:

  .. _code-hello:
  
  .. literalinclude:: srcs/hello.cpp
    :language: cpp

  Our first example.

As you can see in :ref:`hello.cpp <code-hello>` this is one of the most basic ecto cells.

python
------
The python counter part to hello might look like:

.. _code-hello-python:

.. literalinclude:: srcs/hello.py
  :language: python

Lets run it:

.. program-output:: srcs/hello.py
   :in_srcdir: