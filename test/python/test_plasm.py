#!/usr/bin/env python
import ecto
#from ecto.doc import print_module_doc
import buster

def test_plasm():
    scatter = ecto.make(buster.Scatter, n=3, x=3)
    gather = ecto.make(buster.Gather, n=3)
    print "#################\nPlasm test\n#################"
    plasm = ecto.Plasm()
    for f, t in zip(ecto.keys(scatter.outputs), ecto.keys(gather.inputs)):
            plasm.connect(scatter, f, gather, t)
    plasm.go(gather)
    result = gather.o.out.get()
    str = ecto.print_module_doc(scatter)
    print "gather out (should be 9):", result
    assert(result == 9)
    #print plasm.viz()
    #print plasm.vertices()
    #print plasm.edges()
    #ecto.view_plasm(plasm)
    
if __name__ == '__main__':
    test_plasm()




