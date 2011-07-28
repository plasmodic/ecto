tendril
=======

.. _boost::any: http://www.boost.org/doc/libs/1_47_0/doc/html/any.html

The ``tendril`` is a type-erased value container, essentially a
`boost::any`_ (in fact, implemented with ``boost::any``) augmented
with certain conversion rules, conversion operators, introspection
capabilities and the like.  (in fact , each tendril contains a
boost::any), with the ability to automatically perform certain
conversions between the held types.

Overview
--------

You will primarily encounter the ``tendril`` as the right hand side of
the mappings which are the ``tendrils`` objects passed to ecto cell
functions.



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

.. code-block:: c++

	spore<double> val = inputs.at("val");
	val.set_callback(cb);
	if(val() > eps)
		std::cout << val() << std::endl;
	else
		*val = eps;
	val.notify();
	
	spore<Bar> foo = outputs.at("foo");
	foo->spam("hello");
	foo().const_func();

c++ api
-------
.. doxygenclass:: ecto::spore
    :members:
