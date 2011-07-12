#!/usr/bin/env python

import ecto
import ecto_test

def makeplasm(N):
    plasm = ecto.Plasm()
    gen = ecto_test.Generate(start=0, step=1)
    quitter = ecto_test.QuitAfter(N=N)
    plasm.connect(gen[:] >> quitter[:])
    
    return (gen, plasm)

def do_one(N, nthreads):
    print "multithreaded test w/ quit after", N
    (gen, plasm) = makeplasm(N)

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(niter=N+100, nthreads=nthreads)
    
    print "N-threaded actual out: ", gen.outputs.out, " N:", N
    assert N >= (gen.outputs.out - nthreads)
    print "\n" * 5

def do_one_st(N):
    print "multithreaded test w/ quit after", N
    (gen, plasm) = makeplasm(N)

    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=N+100)
    
    print "singlethreaded: actual out:", gen.outputs.out, " N:", N
    assert (N - 1.0) == gen.outputs.out
    print "\n" * 5

for i in range(2,12,2):
    do_one(7, i)
    do_one_st(7)
    do_one(94, i)
    do_one_st(94)
    do_one(191, i)
    do_one_st(191)


