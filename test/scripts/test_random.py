#!/usr/bin/env python
import ecto
import ecto_test
import sys, time


def test_random():
    plasm = ecto.Plasm()

    uni = ecto_test.Uniform01("Random", seed=17)

    printer = ecto_test.Printer("Printy")
    plasm.connect(uni, "out", printer, "in")
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=100)
    while sched.running():
        print "."
        time.sleep(0.1)

if __name__ == '__main__':
    test_random()




