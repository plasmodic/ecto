#!/usr/bin/env python
import ecto
import ecto_test
import sys, time

print "Hardware concurrency is", ecto.hardware_concurrency()


def makeplasm():
    plasm = ecto.Plasm()
    
    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.Sleep("Sleep_0", seconds=0.1)
    sleep1 = ecto_test.Sleep("Sleep_1", seconds=0.1)

    plasm.connect(ping[:] >> sleep0[:],
                  sleep0[:] >> sleep1[:])

    return plasm

def sthreaded():
    p = makeplasm()

    s = ecto.schedulers.Singlethreaded(p)

    assert not s.running()
    
    stime = time.time()
    
    s.execute(niter=3)
    
    assert not s.running()
    etime = time.time()
    assert 0.62 > etime - stime >= 0.6
    print "elapsed:", etime-stime
    stime = time.time()
    s.execute_async(niter=3)
    assert s.running()
    nloops = 0
    while s.running():
        time.sleep(0.01)
        nloops = nloops + 1
    
    assert not s.running()
    assert nloops >= 59
    etime = time.time()
    assert 0.62 > etime - stime >= 0.6
    print "elapsed:", etime-stime

    stime = time.time()
    assert not s.running()
    s.execute_async(niter=3)
    assert s.running()
    s.wait()
    assert not s.running()
    etime = time.time()
    assert 0.62 > etime - stime >= 0.6


kwargs = dict(niter=10)

def tpool():
    def bang(fn):
        p = makeplasm()
        s = ecto.schedulers.Threadpool(p)
        assert not s.running()
        stime = time.time()
        fn(s)
        etime = time.time()
        elapsed = etime-stime
        return elapsed

    def justrun(s):
        print "just run it"
        s.execute(**kwargs)

    runtime = bang(justrun)

    def asyncit(s):
        print "async and wait on running() == false"
        s.execute_async(**kwargs)
        while s.running():
            pass

    asynctime = bang(asyncit)

    def waitit(s):
        print "async and wait()"
        s.execute_async(**kwargs)
        s.wait()
    
    waittime = bang(waitit)

    print runtime, asynctime, waittime

    median = (runtime + asynctime + waittime) / 3.0
    low = median*0.75
    high = median*1.2

    print "lowest acceptable:", low, "highest acceptable:", high
    assert high > runtime > low
    assert high > asynctime > low
    assert high > waittime > low

def tpool_interrupt():
    p = makeplasm()
    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.execute_async()
    time.sleep(0.1)
    s.interrupt()
    s.wait()
    etime = time.time()
    time.sleep(0.1)
    print "elapsed:", etime-stime
    assert etime-stime > 0.1
    assert 0.4 > etime-stime

def tpool_throw_on_double_execute():
    p = makeplasm()
    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.execute_async()
    try: 
        s.execute()
    except Exception, e:
        assert e.message == "threadpool scheduler already running"
        print "as expected:", e.message

def tpool_wait_on_nothing():
    p = makeplasm()
    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.wait()
    etime = time.time()
    print etime-stime
    assert 0.01 > etime-stime

if ecto.hardware_concurrency() > 1:
    tpool()
    tpool_wait_on_nothing()
    tpool_throw_on_double_execute()
    tpool_interrupt()
else:
    print "threadpool async execution tests disabled due to lack of hardware concurrency"

sthreaded()

print "okay."
