#!/usr/bin/env python
import ecto
from ecto.doc import print_module_doc
import buster

def test_sample():
    plasm = ecto.Plasm()
    
    g = ecto.make(buster.Generate,start=0, step=2)    
    m = ecto.make(buster.Multiply, factor=2)
    
    plasm.connect(g, "out", m , "in")
    
    for i in range(0, 11):
        plasm.mark_dirty(g)
        plasm.go(m)
        print "output: ", m.outputs["out"].get()
    assert(m.outputs["out"].get() == 40)
    #ecto.view_plasm(plasm)
    print "finished"
    
if __name__ == '__main__':
    test_sample()
