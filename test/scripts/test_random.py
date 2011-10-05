#!/usr/bin/env python
import ecto
import ecto_test
import sys, time


plasm = ecto.Plasm()

uni = ecto_test.Uniform01("Random", seed=17)
print "OTUPUTS:", uni.outputs
printer = ecto_test.Printer("Printy")

print ecto.version()

plasm.connect(uni, "out", printer, "in")
    
sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute(niter=100)
while sched.running():
    print "."
    time.sleep(0.1)




