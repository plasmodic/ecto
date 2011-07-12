#!/usr/bin/env python
import ecto
import ecto_test
import sys, time

print "Hardware concurrency is", ecto.hardware_concurrency()


def makeplasm():
    plasm = ecto.Plasm()
    
    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.Sleep("Sleep_0", seconds=1.0)
    sleep1 = ecto_test.Sleep("Sleep_1", seconds=1.0)

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
    assert 6.2 > etime - stime >= 6.0
    print "elapsed:", etime-stime
    stime = time.time()
    s.execute_async(niter=3)
    assert s.running()
    nloops = 0
    while s.running():
        time.sleep(0.1)
        nloops = nloops + 1
    
    assert not s.running()
    assert nloops >= 59
    etime = time.time()
    assert 6.2 > etime - stime >= 6.0
    print "elapsed:", etime-stime

    stime = time.time()
    assert not s.running()
    s.execute_async(niter=3)
    assert s.running()
    s.wait()
    assert not s.running()
    etime = time.time()
    assert 6.2 > etime - stime >= 6.0

    
def tpool0():
    p = makeplasm()

    s = ecto.schedulers.Threadpool(p)

    assert not s.running()
    stime = time.time()
    s.execute(niter=3)
    etime = time.time()
    assert not s.running()
    print "elapsed:", etime - stime
    assert etime - stime >= 4.0

    stime = time.time()
    s.execute_async(niter=3)
    time.sleep(0.1)
    assert s.running()
    nloops = 0
    while s.running():
        time.sleep(0.1)
        nloops = nloops + 1

    assert not s.running()
    assert nloops >= 39
    etime = time.time()
    print "elapsed:", etime-stime
    assert 4.1 > etime - stime >= 4.0

def tpool1():

    p = makeplasm()

    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.execute_async(niter=3)
    time.sleep(0.1)
    assert s.running()
    nloops = 0
    while s.running():
        nloops = nloops + 1
        time.sleep(0.1)

    print "nloops=", nloops
    assert not s.running()
    assert nloops >= 39
    etime = time.time()
    print "elapsed:", etime-stime
    assert 4.1 > etime - stime >= 4.0

def tpool2():
    p = makeplasm()

    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.execute_async(niter=3)
    time.sleep(0.1)
    assert s.running()
    s.wait()
    assert not s.running()
    etime = time.time()
    print "elapsed:", etime-stime
    assert 4.1 > (etime - stime) > 4.0

sthreaded()

if ecto.hardware_concurrency() > 1:
    tpool0()
    tpool1()
    tpool2()
else:
    print "threadpool async execution tests disabled due to lack of hardware concurrency"

print "okay."
