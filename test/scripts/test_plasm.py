#!/usr/bin/env python
import i
import ecto
import ecto_test

def test_plasm():
    scatter = ecto_test.Scatter(n=3, x=3)
    gather = ecto_test.Gather(n=3)
    plasm = ecto.Plasm()
    for f, t in zip(scatter.outputs.keys(), gather.inputs.keys()):
        plasm.connect(scatter, f, gather, t)
    #plasm.go(gather)
    #plasm.set_input(scatter)
    #plasm.set_output(gather)
    plasm.execute()
    #ecto.view_plasm(plasm)
    result = gather.outputs.out
    print result
    assert(result == 9) # 3 * 3

if __name__ == '__main__':
    test_plasm()



