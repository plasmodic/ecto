#!/usr/bin/env python
import ecto
import buster

def test_doc():
    scatter = buster.Scatter(n=6, x=3)
    gather = buster.Gather(n=3)
    gather2 = buster.Gather(n=3)
    plasm = ecto.Plasm()
    for t,f in zip(gather.inputs.keys(), scatter.outputs.keys()):
        plasm.connect(scatter, f, gather, t)
    for t,f in zip(gather2.inputs.keys(), scatter.outputs.keys()[3:]):
        plasm.connect(scatter, f, gather2, t)
    plasm.execute()
    result = gather.outputs.out
    assert(result == 9) # 3 * 3
    
    ecto.print_module_doc(scatter)
    ecto.print_module_doc(gather)
    print plasm.viz()
#    ecto.view_plasm(plasm)

if __name__ == '__main__':
    test_doc()

