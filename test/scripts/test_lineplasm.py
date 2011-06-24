#!/usr/bin/env python
import ecto
import ecto_test
import sys


def test_plasm(nthreads, niter, n_nodes, incdelay):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate("Gen", step=1.0, start=0.0)
    inc = ecto_test.Increment("Increment 0", delay=incdelay)

    plasm.connect(gen, "out", inc, "in")

    for j in range(n_nodes-1): # one has already been added
        inc_next = ecto_test.Increment("Increment_%u" % (j+1), delay=incdelay)
        plasm.connect(inc, "out", inc_next, "in")
        inc = inc_next

    printer = ecto_test.Printer("Printy")
    plasm.connect(inc, "out", printer, "in")
    
    o = open('graph.dot', 'w')
    print >>o, plasm.viz()
    o.close()
    print "\n", plasm.viz(), "\n"
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(nthreads, niter)
    print "RESULT:", inc.outputs.out
    shouldbe = float(n_nodes + niter - 1)
    print "expected:", shouldbe
    assert inc.outputs.out == shouldbe

    #sched.execute(nthreads, niter)
    #result = inc.outputs.out
    #print "RESULT:", result
    
    #shouldbe = float(n_nodes + (niter*2 - 1))
    #assert result == shouldbe
                     
    return


if __name__ == '__main__':
    test_plasm(nthreads=int(sys.argv[1]), 
               niter=int(sys.argv[2]), 
               n_nodes=int(sys.argv[3]),
               incdelay=int(sys.argv[4])
               )



