.. _ectodoc:

Automatic Documentation and Ecto's Sphinx Extensions
====================================================

Ecto comes with a number of extensions to assist in documenting
ecto-based projects.  Try clicking "Show Source" on the right for a
before-and-after sphinx view....  there is much less text in the
source for this document than there is on this page.


ectocell
--------

Placing the following command in sphinx::

  .. ectocell:: ecto_test Add
 
Will import the python module ``ecto_test`` into the running Sphinx
process, find the class named ``Add`` and generate documenation,
which looks like this (actual output):

.. ectocell:: ecto_test Add

Cells' docstrings should contain identical information, albeit
surrounded with a bit of noise:

.. literalinclude:: ectotest_docstring.py

and the output below, which is truncated for clarity as there is
quite a bit of noise, starting with ``Methods defined here``.

.. program-output:: doc/source/ectotest_docstring.py | head -35
   :nostderr:


ectoplot
--------

This directive generates and includes graph diagrams.  The directive
takes the name of a file and the name of an object in that file.
Sphinx will execute the file, pull the named object out of the file
(which must be a live plasm), get the ``graphviz`` information from
the plasm, generate and include a graphic.  For instance this direcive::

   .. ectoplot:: sampleplasm.py plasm

Will look for the file ``sampleplasm.py`` in the documentation
directory, execute it, pull out and plot the object named plasm.  If
that script looks like this:

.. literalinclude:: sampleplasm.py

you'll get this:

.. ectoplot:: sampleplasm.py plasm




