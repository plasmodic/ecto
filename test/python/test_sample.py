#!/usr/bin/env python
import ecto
from ecto.doc import print_module_doc
import buster

def m_squared():
    plasm = ecto.Plasm()

    m = ecto.make(buster.Multiply)
    print_module_doc(m)
    m2 = ecto.make(buster.Multiply)
    plasm.connect(m, "out", m2 , "in")
    module = plasm.toModule([m,],[m2,])
    print_module_doc(module)

def test_sample():
    plasm = ecto.Plasm()
    
    g = buster.Generate()
    g.Params(g.params)
    g.Config()
    print_module_doc(g)
    
    m = buster.Multiply()
    m.Params(m.params)
    m.Config()
    print_module_doc(m)
    
    plasm.connect(g, "out", m , "in")
    
    print plasm.viz()
    for i in range(0, 10):
        plasm.markDirty(g)
        plasm.go(m)
        print "output: ", m.outputs["out"].get()
    ecto.view_plasm(plasm)
    print "finished"
    
if __name__ == '__main__':
    m_squared()
    test_sample()
