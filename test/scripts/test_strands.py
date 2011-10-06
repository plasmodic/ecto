#!/usr/bin/env python
import ecto
import ecto_test

def test_strands(nlevels, SchedType, execfn, expect):
    s1 = ecto.Strand()
    s2 = s1
    s3 = ecto.Strand()
    
    print "s1.id ==", s1.id
    print "s2.id ==", s2.id
    print "s3.id ==", s3.id
    assert s1.id == s2.id
    assert s3.id != s2.id
    assert s3.id != s1.id
    
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=1.0)
    noncurr = ecto_test.DontCallMeFromTwoThreads(strand=s1)
    plasm.connect(gen, "out", noncurr, "in")

    for k in range(nlevels):
        next = ecto_test.DontCallMeFromTwoThreads(strand=s1)
        plasm.connect(noncurr, "out", next, "in")
        noncurr = next

    printer = ecto_test.Printer()
    plasm.connect(noncurr, "out", printer, "in")
    
    sched = SchedType(plasm)
    print "sched=", sched
    execfn(sched)

    result = noncurr.outputs.out
    print "result=", result
    assert(result == expect)

def test_implicit_strands(nlevels, SchedType, execfn, expect):
    
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=1.0)
    noncurr = ecto_test.CantCallMeFromTwoThreads()
    plasm.connect(gen, "out", noncurr, "in")

    for k in range(nlevels):
        next = ecto_test.CantCallMeFromTwoThreads()
        plasm.connect(noncurr, "out", next, "in")
        noncurr = next

    printer = ecto_test.Printer()
    plasm.connect(noncurr, "out", printer, "in")
    
    sched = SchedType(plasm)
    print "sched=", sched
    execfn(sched)

    result = noncurr.outputs.out
    print "result=", result
    assert(result == expect)

def shouldfail():
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=1.0)
    nc1 = ecto_test.DontCallMeFromTwoThreads()
    plasm.connect(gen, "out", nc1, "in")

    nc2 = ecto_test.DontCallMeFromTwoThreads()
    plasm.connect(nc1, "out", nc2, "in")

    printer = ecto_test.Printer()
    plasm.connect(nc2, "out", printer, "in")
    
    sched = ecto.schedulers.Threadpool(plasm)
    try:
        print "about to execute... this should throw"
        sched.execute(nthreads=4, niter=4)
        util.fail()
    except RuntimeError, e:
        print "good, python caught error", e

shouldfail()
print "shouldfail passed"

test_implicit_strands(4, ecto.schedulers.Threadpool, lambda s: s.execute(nthreads=4, niter=4), expect=4.0)
test_implicit_strands(4, ecto.schedulers.Singlethreaded, lambda s: s.execute(niter=4), expect=4.0)

test_strands(4, ecto.schedulers.Singlethreaded, lambda s: s.execute(niter=4), expect=4.0)
test_strands(4, ecto.schedulers.Threadpool, lambda s: s.execute(nthreads=4, niter=4), expect=4.0)


