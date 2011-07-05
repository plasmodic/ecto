#!/usr/bin/env python
#
#  This script really just generates load, not actually used in testing
#
import ecto
import ecto_test
import sys

def build_crunchgraph(nlevels, ncalls):
    
    plasm = ecto.Plasm()
    prevlevel = [ecto_test.Add("Adder 0_%u" % x) for x in range(2**(nlevels-1))]
    for adder in prevlevel:
        plasm.connect(ecto_test.Uniform01(ncalls=ncalls)["out"] >> adder["left"],
                      ecto_test.Uniform01(ncalls=ncalls)["out"] >> adder["right"])

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

    return (plasm, final_adder)

def test_plasm(nlevels, ncalls, niter):
    (plasm, outnode) = build_crunchgraph(nlevels, ncalls)

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
    print "RESULT:", outnode.outputs.out

if __name__ == '__main__':
    test_plasm(nlevels=6, ncalls=10000, niter=6000)




