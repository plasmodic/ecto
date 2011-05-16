#!/usr/bin/env python
import ecto
import buster

def test_doc():
    scatter = buster.Scatter(n=3, x=3)
    gather = buster.Gather(n=3)
    plasm = ecto.Plasm()
    for f, t in zip(scatter.outputs.keys(), gather.inputs.keys()):
        plasm.connect(scatter, f, gather, t)
    plasm.set_input(scatter)
    plasm.set_output(gather)
    plasm.execute()
    result = gather.outputs.out
    assert(result == 9) # 3 * 3
    
    ecto.print_module_doc(scatter)
    ecto.print_module_doc(gather)
    print plasm.viz()
    #ecto.view_plasm(plasm)

if __name__ == '__main__':
    test_doc()

