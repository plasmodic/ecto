#!/usr/bin/env python
import ecto
import ecto_test
import pyecto

def test_doc():
    scatter = ecto_test.Scatter(n=6, x=3)
    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=3)
    plasm = ecto.Plasm()
    for t,f in zip(gather.inputs.keys(), scatter.outputs.keys()):
        plasm.connect(scatter, f, gather, t)
    for t,f in zip(gather2.inputs.keys(), scatter.outputs.keys()[3:]):
        plasm.connect(scatter, f, gather2, t)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=1)
    result = gather.outputs.out
    assert(result == 9) # 3 * 3
    
    ecto.print_module_doc(scatter)
    ecto.print_module_doc(gather)
    print plasm.viz()
    
    #ecto.view_plasm(plasm)
def test_inspection():
    ecto.list_ecto_module(ecto_test)
    print ecto.list_ecto_module(pyecto)

if __name__ == '__main__':
    test_doc()
    test_inspection()
