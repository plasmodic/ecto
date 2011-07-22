#!/usr/bin/env python
import ecto
import ecto_test
import sys


fname = "test_redirect.log"

def make(Schedtype):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate("Gen", step=1.0, start=0.0)

    printer = ecto_test.Printer("Printy")
    ecto.log_to_file(fname)
    plasm.connect(gen[:] >> printer[:])
    return Schedtype(plasm)

def verify():
    f = open(fname)
    txt = f.read()
    print ">>>", txt, "<<<"
    assert len(txt.splitlines()) >= 5
    

sched = make(ecto.schedulers.Singlethreaded)
sched.execute(niter=5)
verify()

sched = make(ecto.schedulers.Singlethreaded)
sched.execute_async(niter=5)
sched.wait()
verify()



sched = make(ecto.schedulers.Threadpool)
sched.execute(niter=5)
verify()

sched = make(ecto.schedulers.Threadpool)
sched.execute_async(niter=5)
sched.wait()
verify()


