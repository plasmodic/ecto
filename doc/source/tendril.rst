tendril
=======

A tendril is the slender, winding organ of the
ecto::cell that gives it its awesome type erasure and uber
flexibility.

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
