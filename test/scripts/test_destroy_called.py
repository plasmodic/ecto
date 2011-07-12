#!/usr/bin/env python

import ecto
import ecto_test

def do_one(quitcount, runcount, nthreads, expect):
    print "test w/ quit after", quitcount
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(start=0, step=1)
    quitafter = ecto_test.QuitAfter(N=quitcount)
    destroyer = ecto_test.FlagOnDestroy()

    plasm.connect(gen[:] >> destroyer[:],
                  destroyer[:] >> quitafter[:])
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(niter=runcount, nthreads=nthreads)
    
    print "actual out:", destroyer.outputs.out
    assert destroyer.outputs.out == expect

do_one(quitcount=24, runcount=100, nthreads=4, expect=1.0)
do_one(quitcount=150, runcount=100, nthreads=4, expect=0.0)

#for i in range(2,12,2):
#    do_one(7,i)
    #do_one(94,i)
    #do_one(191,i)


