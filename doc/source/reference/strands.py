#!/usr/bin/env python
import ecto
import ecto_test

s1 = ecto.Strand()

print "s1.id ==", s1.id

plasm = ecto.Plasm()
gen = ecto_test.Generate(step=1.0, start=1.0)
noncurr = ecto_test.DontCallMeFromTwoThreads(strand=s1)
plasm.connect(gen, "out", noncurr, "in")

for k in range(3):
    next = ecto_test.DontCallMeFromTwoThreads(strand=s1)
    plasm.connect(noncurr, "out", next, "in")
    noncurr = next

printer = ecto_test.Printer()
plasm.connect(noncurr, "out", printer, "in")

sched = ecto.schedulers.Threadpool(plasm)
sched.execute(niter=2,nthreads=8)