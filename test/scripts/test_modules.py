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
def test_modules_spec():
    g = ecto_test.Generate(start=0, step=2)
    x = g["out"]
    x = g["out","out"]
    try:
        x = g[2.0]
        assert False, "should have thrown"
    except TypeError, e:
        print e
    try:
        x = g["out",2.0]
        assert False, "should have thrown"
    except RuntimeError, e:
        print e
    try:
        x = g["out","and","about"]
        assert False, "should have thrown"
    except RuntimeError, e:
        print e
    
    scatter = ecto_test.Scatter(n=3, x=3)
    gather = ecto_test.Gather(n=3)
    a = scatter[scatter.outputs.keys()]
    b = gather[gather.inputs.keys()]
    print a,b
    print a >> b
    plasm = ecto.Plasm()
    plasm.connect(a>>b)
    plasm.execute(1)
    result = gather.outputs.out
    print result
    assert(result == 9) # 3 * 3
    
    connections = scatter[:] >> gather[:]
    assert(len(connections) == 3)
    try:
        scatter[1:-1]
        assert False, "[1:-1] should not work..."
    except RuntimeError,e:
        print e
        

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

def do_fail(x,exception_type = RuntimeError ,args = None):
    try:
        x(args)
        assert False
    except exception_type,e:
        print "good, caught error:", e
    finally:
        print "unknown exception"
        
def too_many_positionalargs(_):
    ecto_test.Generate("foo", "bar")

def type_and_instance_names():
    m = ecto_test.Generate()
    name = m.name()
    print "name is:", name
    assert name.startswith("ecto_test::Generate<double>")

    t = m.type_name()
    print "type is:", t

    m2 = ecto_test.Generate("user-supplied name")
    assert m2.name() == "user-supplied name"
    print "m2.type_name =", m2.type_name() 
    assert m2.type_name() == "ecto_test::Generate<double>"
    
def not_allocable():
    d = ecto_test.DontAllocateMe()

def test_modules_wrong_args():
    not_allocable()
    do_fail(noarg)
    g = ecto_test.Generate()
    do_fail(wrong_type,ecto.TypeMismatch,g)
    do_fail(already_set,Exception,g)
    do_fail(too_many_positionalargs, RuntimeError)
    novel_sets(g)
    right_type(g)

    type_and_instance_names()

if __name__ == '__main__':
    test_modules_01()
    test_modules_wrong_args()
    test_modules_spec()
