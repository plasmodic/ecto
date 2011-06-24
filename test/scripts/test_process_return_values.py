#!/usr/bin/env python

import ecto
import ecto_test

def do_one(N, nth):
    print "test w/ quit after", N
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(start=0, step=1)
    quitter = ecto_test.QuitAfter(N=N)
    plasm.connect(gen[:] >> quitter[:])
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(nthreads=nth, niter=N+100)
    
    print "actual out:", gen.outputs.out, " N:", N
    assert N >= gen.outputs.out
    print "\n" * 5


for i in range(2,12,2):
    do_one(7,i)
    do_one(94,i)
    do_one(191,i)


