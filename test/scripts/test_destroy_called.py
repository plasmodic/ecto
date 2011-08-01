#!/usr/bin/env python

import ecto
import ecto_test
def do_one1(quitcount, runcount, Scheduler):
    plasm = ecto.Plasm()
    gen = ecto_test.Generate(start=0, step=1)
    quitafter = ecto_test.QuitAfter(N=quitcount)
    destroyer = ecto_test.FlagOnDestroy()

    plasm.connect(gen[:] >> destroyer[:],
                  destroyer[:] >> quitafter[:])
    
    sched = Scheduler(plasm)
    sched.execute(niter=runcount)
    out = destroyer.outputs.at("out")
    print "in scope out:", destroyer.outputs.out
    assert destroyer.outputs.out == 0
    return out
def do_one(quitcount, runcount, Scheduler):
    print "test w/ quit after", quitcount
    out = do_one1(quitcount,runcount,Scheduler)
    print "post out:", out.val
    assert  out.val == 1

do_one(quitcount=24, runcount=100, Scheduler=ecto.schedulers.Threadpool)
do_one(quitcount=24, runcount=100, Scheduler=ecto.schedulers.Singlethreaded)

do_one(quitcount=150, runcount=100, Scheduler=ecto.schedulers.Threadpool)
do_one(quitcount=150, runcount=100, Scheduler=ecto.schedulers.Singlethreaded)

#for i in range(2,12,2):
#    do_one(7,i)
    #do_one(94,i)
    #do_one(191,i)


