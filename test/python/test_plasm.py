#!/bin/python

import ecto
#from ecto.doc import printModuleDoc
import buster

def test_plasm():
    scatter = ecto.make(buster.Scatter)
    gather = ecto.make(buster.Gather)
    print "#################\nPlasm test\n#################"
    plasm = ecto.Plasm()
    for f,t in zip(ecto.keys(scatter.outputs),ecto.keys(gather.inputs)):
            plasm.connect(scatter, f, gather, t)    
    plasm.go(gather)
    print "gather out:", gather.outputs["out"].get()

if __name__ == '__main__':
    test_plasm()




