#!/usr/bin/env python
import ecto
import ecto_test

def test_dual_line_plasm(nlevels):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=0.0)
    incl, incr = ecto_test.Increment(), ecto_test.Increment()

    plasm.connect(gen, "out", incl, "in")
    plasm.connect(gen, "out", incr, "in")

    for j in range(nlevels-1): # one set of incs has already been added
        print j
        inc_nextl, inc_nextr = ecto_test.Increment(), ecto_test.Increment()
        plasm.connect(incl, "out", inc_nextl, "in")
        plasm.connect(incr, "out", inc_nextr, "in")
        incl, incr = inc_nextl, inc_nextr

    add = ecto_test.Add()
    plasm.connect(incl, "out", add, "left")
    plasm.connect(incr, "out", add, "right")
    printer = ecto_test.Printer()
    plasm.connect(add, "out", printer, "in")
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(niter=1, nthreads=int(nlevels/2))
    result = add.outputs.out
    print "result=", result
    assert(result == nlevels * 2)

    sched.execute(niter=2, nthreads=int(nlevels/2))
    result = add.outputs.out
    print "iter2 result=", result
    assert result == (nlevels + 2) * 2

    sched.execute(niter=3, nthreads=int(nlevels/2))
    result = add.outputs.out
    print "iter3 result=", result
    assert result == (nlevels + 5) * 2

if __name__ == '__main__':
    test_dual_line_plasm(10)
    test_dual_line_plasm(100)



