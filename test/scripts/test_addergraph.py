#!/usr/bin/env python
import ecto
import ecto_test
import sys

def build_addergraph(nlevels):
    
    plasm = ecto.Plasm()
    generators = [ecto_test.Generate("Generator 0_%u" % x, step=1.0, start=0.0)
                  for x in range(2**nlevels)]
    prevlevel = [ecto_test.Add("Adder 0_%u" % x) for x in range(2**(nlevels-1))]
    for adder in prevlevel:
        plasm.connect(
                      ecto_test.Generate("Generator", step=1.0, start=1.0)["out"] >> adder["left"],
                      ecto_test.Generate("Generator", step=1.0, start=1.0)["out"] >> adder["right"]
                      )

    print "prev has", len(prevlevel)

    for k in range(nlevels-2, -1, -1):
        print "****** k=", k, " ***********"
        thislevel = [ecto_test.Add("Adder %u_%u" % (k, x)) for x in range(2**k)]
        print "prevlevel=", prevlevel
        print "thislevel=", thislevel
        index = 0
        print "for...", range(2**k)
        for r in range(2**k):
            print "prev[%u] => cur[%u]" % (index, r)
            plasm.connect(prevlevel[index]["out"] >> thislevel[r]["left"])
            index += 1
            print "prev[%u] => cur[%u]" % (index, r)
            plasm.connect(prevlevel[index]["out"]>>thislevel[r]["right"])
            index += 1
        prevlevel = thislevel

    assert len(prevlevel) == 1
    final_adder = prevlevel[0]
    printer = ecto_test.Printer("printy!")
    plasm.connect(final_adder, "out", printer, "in")
    return (plasm, final_adder)

def test_plasm(nlevels, nthreads, niter):
    (plasm, outnode) = build_addergraph(nlevels)

    #o = open('graph.dot', 'w')
    #print >>o, plasm.viz()
    #o.close()
    #print "\n", plasm.viz(), "\n"
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(nthreads, niter)
    print "RESULT:", outnode.outputs.out
    shouldbe = float(2**nlevels * niter)
    print "expected:", shouldbe
    assert outnode.outputs.out == shouldbe

if __name__ == '__main__':
    test_plasm(1, 1, 1)
    test_plasm(1, 1, 2)
    test_plasm(8, 1, 5)
    test_plasm(9, 64, 100)
    test_plasm(10, 8, 10)
    test_plasm(11, 8, 10)




