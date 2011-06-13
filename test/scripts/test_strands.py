#!/usr/bin/env python
import ecto
import ecto_test

s1 = ecto.Strand()
s2 = s1
s3 = ecto.Strand()

print "s1.id ==", s1.id
print "s2.id ==", s2.id
print "s3.id ==", s3.id
assert s1.id == s2.id
assert s3.id != s2.id
assert s3.id != s1.id

def test_strands(nlevels, SchedType, execfn, expect):
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
    
    o = open('graph.dot', 'w')
    print >>o, plasm.viz()
    o.close()
    print "\n", plasm.viz(), "\n"
    
    sched = SchedType(plasm)
    print "sched=", sched
    execfn(sched)

    result = noncurr.outputs.out
    print "result=", result
    assert(result == expect)

if __name__ == '__main__':
    # test_strands(10, ecto.schedulers.Singlethreaded, lambda s: s.execute(niter=5), expect=5.0)

    test_strands(10, ecto.schedulers.Threadpool, lambda s: s.execute(nthreads=5, niter=5), expect=5.0)
    #sched = ecto.schedulers.Threadpool(plasm)
    #sched.execute(nthreads=int(nlevels), niter=5)

