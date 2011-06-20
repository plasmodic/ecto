#!/usr/bin/env python
import ecto
import ecto_test
import sys


def test_random():
    plasm = ecto.Plasm()

    uni = ecto_test.Uniform01("Random", seed=17)

    printer = ecto_test.Printer("Printy")
    plasm.connect(uni, "out", printer, "in")
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(10)


if __name__ == '__main__':
    test_random()




