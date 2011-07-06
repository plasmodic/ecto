tendril
============================================
A tendril is the slender, winding organ of the
ecto::module that gives it its awesome type erasure and uber
flexibility.

.. autoclass:: ecto.Tendril
    :members:

.. autoclass:: ecto.Tendrils

python api
----------
  dirty
       Has the tendril changed since the last time?
   
  doc
       A doc string that describes the purpose of this tendril.
   
  has_default
       Does the tendril have an explicit default value?
       Remember that the implicit default is always the default constructed type.
   
  required
       Is this tendril required to be connected?
   
  type_name
       The type of the value held by the tendril.
   
  user_supplied
       Has the value been set by the user?
       
  val
       The value held by the tendril.
       It requires boost::python bindings to be accessible from python.
       If none are available it will be None.


spore
======
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
