#!/usr/bin/env python
import ecto
import ecto_test
import sys

def test_plasm(nthreads, niter, n_nodes):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=0.0)
    inc = ecto_test.Increment(delay=100)

    plasm.connect(gen, "out", inc, "in")

    for j in range(n_nodes): # one set of incs has already been added
        print j
        inc_next = ecto_test.Increment(delay=100)
        plasm.connect(inc, "out", inc_next, "in")
        inc = inc_next

    printer = ecto_test.Printer()
    plasm.connect(inc, "out", printer, "in")
    
    o = open('graph.dot', 'w')
    print >>o, plasm.viz()
    o.close()
    print "\n", plasm.viz(), "\n"
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(int(nthreads), int(niter))
    # sched.execute(1, 5)
    return


if __name__ == '__main__':
    test_plasm(sys.argv[1], sys.argv[2], int(sys.argv[3])-1)



