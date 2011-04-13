

#!/usr/bin/env python
import ecto
#from ecto.doc import print_module_doc
import buster

def test_plasm():
    scatter = buster.Scatter(n=3, x=3)
    print scatter, dir(scatter), scatter.outputs #, scatter.o
    gather = buster.Gather(n=3)
    print "#################\nPlasm test\n#################"
    plasm = ecto.Plasm()
    print "KEEZ:", scatter.outputs.keys()
    for f, t in zip(scatter.outputs.keys(), gather.inputs.keys()):
        plasm.connect(scatter, f, gather, t)
    plasm.go(gather)
    print dir(gather)
    # print gather.outputs, gather.o
    # result = gather.o.out.get()
    result = gather.outputs.out
    str = ecto.print_module_doc(scatter)
    print "gather out (should be 9):", result
    assert(result == 9)
    print plasm.viz()
    print plasm.vertices()
    print plasm.edges()
    #ecto.view_plasm(plasm)
    
if __name__ == '__main__':
    test_plasm()



