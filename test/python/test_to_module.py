#!/usr/bin/env python

import ecto, buster

def makeplasm():
    plasm = ecto.Plasm()
    g = buster.Generate(step=3,start=4)
    m = buster.Multiply(factor=13)
    plasm.connect(g, 'out', m, 'in')
    m2 = buster.Multiply(factor=2)
    plasm.connect(m, 'out', m2, 'in')
    return plasm

def makemodule():
    plasm = ecto.Plasm()
    g = buster.Generate(step=3,start=4)
    m = buster.Multiply(factor=2)
    plasm.connect(g, 'out', m, 'in')
    m2 = buster.Multiply(factor=2)
    plasm.connect(m, 'out', m2, 'in')
    return plasm.to_module([g],[m2])

def makemodule2():
    plasm = ecto.Plasm()
    m = buster.Multiply(factor=5)
    m2 = buster.Multiply(factor=3)
    plasm.connect(m, 'out', m2, 'in')
    return plasm.to_module([m],[m2])

def test_make_modules():
    plasm = ecto.Plasm()
    m1 = makemodule()
    m2 = makemodule2()
    plasm.connect(m1,"out",m2,"in")
    for _i in range(3):
        plasm.mark_dirty(m1)
        plasm.go(m2)
        print m1.outputs["out"].get()
        print m2.outputs["out"].get()
    #ecto.view_plasm(plasm)
if __name__ == "__main__":
    test_make_modules()
