#!/usr/bin/env python
import ecto
import ecto_test

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
    t1 = t2
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
    x = ecto_test.make_pod_tendril()
    x.val = 10
    t1 = ecto.Tendril()
    #this connection should force the t1 to become a native type.
    t1 = x
    t1.val = 20
    assert t1.type_name == x.type_name
    assert x.val == 20
    
if __name__ == '__main__':
    test_tendril()
    test_tendril_defs()
    test_cpp_python_tendril()
