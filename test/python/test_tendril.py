#!/usr/bin/env python
import ecto
import buster
#  bp::class_<tendril>("Tendril")
#    .def("connect", &tendril::connect)
#    .add_property("doc",&tendril::doc,&tendril::setDoc)
#    .add_property("type_name", &tendril::type_name)
#    .add_property("val", &tendril::extract,(void(tendril::*)(bp::object)) &tendril::set)
#    .def("get",&tendril::extract)
#    .def("set",(void(tendril::*)(bp::object)) &tendril::set)
def test_tendril():
    tendril = ecto.Tendril()
    tendril.set(5)
    t = ecto.Tendril()
    t.val = 5
    assert t.val == t.get()
    assert t.val == 5
    assert tendril.val == t.val
    assert tendril.get() == 5
    assert tendril.get() == t.get()
    tendril.val = 10
    t.val = tendril.val
    t.val = "hi"
    assert tendril.val != "hi"
    assert t.val == t.get()
    assert t.val == "hi"
    
def test_tendril_defs():
    t1 = ecto.Tendril()
    t2 = ecto.Tendril()
    t1.val = 10
    t1.connect(t2)
    assert t2.val == t1.val
    t2.val = 13
    assert t1.val == 13
    print t1.doc
    print t1.type_name
    print t1.val
    print t1.get()
    t1.set("foo")
    print t1.val
    assert t2.val == t1.val

def test_cpp_python_tendril():
    x = buster.make_pod_tendril()
    print x.val
    x.val = 10
    print x.val
    print x.type_name
    t1 = ecto.Tendril()
    print t1.type_name
    t1.connect(x)
    t1.val = 20
    print t1.type_name
    print x.type_name
    print x.val
    
if __name__ == '__main__':
    test_tendril()
    test_tendril_defs()
    test_cpp_python_tendril()