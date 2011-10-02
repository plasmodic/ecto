.. _tendril:

tendril
=======

.. _boost::any: http://www.boost.org/doc/libs/1_47_0/doc/html/any.html

The ``tendril`` is a type-erased value container, essentially a
`boost::any`_ (in fact, implemented with ``boost::any``) augmented
with certain conversion rules, operators, introspection capabilities,
docstrings and the like.  

Overview
--------

You will primarily encounter the ``tendril`` as the right hand side of
the mappings which are the ``tendrils`` objects passed to ecto cell
functions, that is, a tendrils object is essentially a map of
``std::string`` -> ``shared_ptr<tendril>``.  Each ecto cell has
associated with it three sets of tendrils: parameters, input, and
output.  The *scheduler*, external to the cell, copies data between
one cell's output tendril and the input tendril of another cell.

.. literalinclude:: ../src/Example01.cpp
   :language: cpp
   :start-after: using
   :end-before: ECTO_CELL

See :download:`../src/Example01.cpp`

A script like this,

.. literalinclude:: ../src/Example01.py
   :language: py

Will output,

.. program-output:: ../src/Example01.py
   :prompt:
   :in_srcdir:

.. _tendril-conversions:
  
Tendril Conversions
^^^^^^^^^^^^^^^^^^^

The table below explains (somewhat) the type conversions that happen
when one tendril or type is inserted to another.

.. rubric:: Tendril Conversions

+----------+--------------------+------------------------------------+------------+--------+
| FROM     |                    |                                    |            |        |
|          |                    | python                             |            |        |
| TO       | ``none``           | object                             | ``T``      | ``U``  |
+----------+--------------------+------------------------------------+------------+--------+
| ``none`` | assignment (no-op) | python                             | ``T``      | ``U``  |
|          |                    | object                             |            |        |
|          |                    |                                    |            |        |
+----------+--------------------+------------------------------------+------------+--------+
| python   | ValueNone error    | assignment                         | python     | python |
| object   |                    |                                    | object     | object |
|          |                    |                                    |            |        |
|          |                    |                                    |            |        |
+----------+--------------------+------------------------------------+------------+--------+
| ``T``    | ValueNone error    | ``T`` via                          | assignment |        |
|          |                    | extract<> or                       |            | error  |
|          | |conversion error| |                                    |            |        |
+----------+--------------------+------------------------------------+------------+--------+
| ``U``    | ValueNone error    | ``U`` via |TypeMismatch|assignment |            |        |
|          |                    | extract<> or                       | error      |        |
|          | |conversion error| |                                    |            |        |
+----------+--------------------+------------------------------------+------------+--------+



python api
----------
.. autoclass:: ecto.Tendril
    :members:

c++ api
-------
.. doxygenclass:: ecto::tendril
    :members:



spore
=====

A spore is a typed handle for a tendril. It is best used in conjunction with tendrils.

c++ api
-------
.. doxygenclass:: ecto::spore
    :members:

.. _tendrils:

tendrils
========

tendrils are containers for the tendril type, essentially a mapping from name to tendril.
The tendrils also give a convenient form of templated type safe access to the data that
tendril objects hold.

python api
----------
.. autoclass:: ecto.Tendrils

c++ api
-------
.. doxygenclass:: ecto::tendrils
    :members:
