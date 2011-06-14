#!/usr/bin/env python
import ecto
import ecto_test

def test_modules_01():
    g = ecto_test.Generate(start=0, step=2)
    g.process()
    assert g.outputs.out == 0
    g.process()
    assert g.outputs.out == 2
    g.configure()
    g.outputs.out = 7.0
    g.process()
    assert  g.outputs.out == 9
    s = ecto_test.Scatter(n = 4, x=3)
    s.process()
    assert(len(s.outputs) == 4)
    for out in s.outputs:
        print out[1].val
        assert(out[1].val == 3)

def noarg(x):
    ecto_test.Generate(start=0, n=3, step=2)

def wrong_type(g):
    g.outputs.out = "hello"

def right_type(g):
    g.outputs.out = 24.5
    assert g.outputs.out == 24.5

def already_set(g):
    g.outputs.declare("out","doc","str")
    print g.outputs.out

def novel_sets(g):
    g.outputs.declare("out2","doc",1.0)
    assert g.outputs.out2 == 1.0
    
    g.inputs.declare("in2","doc","hello")
    assert g.inputs.in2 == "hello"

def do_fail(x,args = None):
    try:
        x(args)
        assert False
    except RuntimeError,e:
        print "good, caught error:", e
        
def too_many_positionalargs(_):
    ecto_test.Generate("foo", "bar")

def type_and_instance_names():
    m = ecto_test.Generate()
    name = m.name()
    print "name is:", name
    assert name.startswith("ecto_test::Generate<double> @ ")

    t = m.type_name()
    print "type is:", t

    m2 = ecto_test.Generate("user-supplied name")
    assert m2.name() == "user-supplied name"
    print "m2.type_name =", m2.type_name() 
    assert m2.type_name() == "ecto_test::Generate<double>"
    
def not_allocable():
    try:
        d = ecto_test.DontAllocateMe()
        assert False, "that should have thrown"
    except:
        print "threw, okay"

def test_modules_wrong_args():
    not_allocable()
    do_fail(noarg)
    g = ecto_test.Generate()
    do_fail(wrong_type,g)
    do_fail(already_set,g)
    do_fail(too_many_positionalargs)
    novel_sets(g)
    right_type(g)

    type_and_instance_names()

if __name__ == '__main__':
    test_modules_01()
    test_modules_wrong_args()
