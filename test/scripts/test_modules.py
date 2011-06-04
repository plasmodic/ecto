#!/usr/bin/env python
import i
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
        
def test_modules_wrong_args():
    do_fail(noarg)
    g = ecto_test.Generate()
    do_fail(wrong_type,g)
    do_fail(already_set,g)
    novel_sets(g)
    right_type(g)
    
if __name__ == '__main__':
    test_modules_01()
    test_modules_wrong_args()
