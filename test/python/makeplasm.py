#!/usr/bin/env python

import ecto, buster

def makeplasm():
    plasm = ecto.Plasm()
    g = ecto.make(buster.Generate, step=3,start=4)
    m = ecto.make(buster.Multiply,factor=13)
    plasm.connect(g, 'out', m, 'in')
    m2 = ecto.make( buster.Multiply, factor=2)
    plasm.connect(m, 'out', m2, 'in')
    return plasm

def makemodule():
    plasm = ecto.Plasm()
    g = ecto.make(buster.Generate, step=3,start=4)
    m = ecto.make(buster.Multiply,factor=2)
    plasm.connect(g, 'out', m, 'in')
    m2 = ecto.make( buster.Multiply, factor=2)
    plasm.connect(m, 'out', m2, 'in')
    return plasm.toModule([],[m2])

def makemodule2():
    plasm = ecto.Plasm()
    m = ecto.make(buster.Multiply,factor=5)
    m2 = ecto.make( buster.Multiply, factor=3)
    plasm.connect(m, 'out', m2, 'in')
    return plasm.toModule([m],[m2])

if __name__ == "__main__":
    plasm = ecto.Plasm()
    m1 = makemodule()
    m2 = makemodule2()
    plasm.connect(m1,"out",m2,"in")
    for i in range(1):
        plasm.markDirty(m1)
        plasm.go(m2)
        print m1.outputs["out"].get()
        print m2.outputs["out"].get()
    ecto.view_plasm(plasm)