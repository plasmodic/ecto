#!/usr/bin/env python
import ecto
import buster

def exemplar():
    plasm = ecto.Plasm()
    
    g = buster.Generate(start=0, step=2)    
    m = buster.Multiply(factor=2)
    
    plasm.connect(g, "out", m , "in")
    
    for i in range(0, 11):
        plasm.mark_dirty(g)
        plasm.go(m)
        print "output: ", m.outputs["out"].get()
    assert m.outputs.out == 40
    #ecto.view_plasm(plasm)
    print "finished"
    
if __name__ == '__main__':
    exemplar()
