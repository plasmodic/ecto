#!/usr/bin/env python
# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
import ecto
import ecto_test
from util import fail
import sys, time

print "Hardware concurrency is", ecto.hardware_concurrency()

eps = 0.1

def makeplasm():
    plasm = ecto.Plasm()

    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.Sleep("Sleep_0", seconds=0.1)
    sleep1 = ecto_test.Sleep("Sleep_1", seconds=0.1)

    plasm.connect(ping[:] >> sleep0[:], sleep0[:] >> sleep1[:])

    return plasm

def sthreaded():
    p = makeplasm()

    s = ecto.schedulers.Singlethreaded(p)

    assert not s.running()

    stime = time.time()

    s.execute(niter=3)

    assert not s.running()
    etime = time.time()
    elapsed = etime-stime
    print "elapsed:", elapsed
    assert 0.6 < elapsed and elapsed <= 0.7 #for slow vms
    stime = time.time()
    s.execute_async(niter=3)
    assert s.running()
    nloops = 0
    while s.running():
        time.sleep(0.01)
        nloops = nloops + 1

    assert not s.running()
    print "nloops=", nloops
    assert nloops >= 30
    etime = time.time()
    elapsed = etime-stime
    print "elapsed:", elapsed
    assert 0.6 < elapsed and elapsed <= 0.7 #for slow vms

    stime = time.time()
    assert not s.running()
    s.execute_async(niter=3)
    assert s.running()
    s.wait()
    assert not s.running()
    etime = time.time()
    elapsed = etime-stime
    print "elapsed:", elapsed
    assert 0.6 < elapsed and elapsed <= 0.7 #for slow vms


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
        s.wait()                            

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
    print "runtime:", runtime, "asynctime", asynctime, "waittime", waittime
    assert high > runtime > low
    assert high > asynctime > low
    assert high > waittime > low

def tpool_interrupt():
    p = makeplasm()
    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.execute_async()
    time.sleep(eps)
    s.interrupt()
    s.wait()
    etime = time.time()
    time.sleep(eps)
    print "elapsed:", etime-stime
    assert etime-stime > eps
    assert 0.4 > etime-stime

def tpool_throw_on_double_execute():
    p = makeplasm()
    s = ecto.schedulers.Threadpool(p)

    stime = time.time()
    s.execute_async(niter=10)
    try:
        print "trying execute..."
        s.execute()
        fail("sched already running")

    except ecto.EctoException, e:
        print "OK:", e
        assert "threadpool scheduler already running" in str(e)
    try:
        print "trying execute_async again"
        s.execute_async()
        fail("sched already running")

    except ecto.EctoException, e:
        print "OK:", e
        assert "threadpool scheduler already running" in str(e)

def do_test(fn):
    def impl(Sched):
        times = { ecto.schedulers.Singlethreaded : 1.0,
                  ecto.schedulers.Multithreaded : 0.6 }
        print "*"*80, "\n", fn.__name__, Sched.__name__
        p = makeplasm()
        s = Sched(p)
        t = times[Sched]
        print "Expecting finish in", t, "seconds"
        fn(s, times[Sched])
    map(impl, ecto.test.schedulers)

def sync(s, ex):
    t = time.time()
    assert not s.running()
    print "starting"
    s.execute(niter=5)
    dur = time.time() - t
    print "done after", dur
    assert dur > ex
    assert dur < (ex + eps)
    assert not s.running()

do_test(sync)

def synctwice(s, ex):
    t = time.time()
    assert not s.running()
    print "starting"
    s.execute(niter=5)
    s.execute(niter=5)
    dur = time.time() - t
    print "done after", dur
    assert dur > (ex*2)
    assert dur < ((ex*2) + eps)
    assert not s.running()
do_test(synctwice)

def ex_async_twice(s, ex):
    s.execute_async(niter=5)
    print "once..."
    assert s.running()
    t = time.time()
    try:
        print "twice..."
        s.execute_async(niter=5)
        fail("that should have thrown")
    except ecto.EctoException, e:
        print "okay, threw"
        print "whee"
    s.wait()
    elapsed = time.time() - t
    assert elapsed > ex
    assert elapsed < (ex + eps)
    
do_test(ex_async_twice)

def wait_on_nothing(s, ex):
    stime = time.time()
    assert not s.running()
    s.wait()
    assert not s.running()
    etime = time.time()
    print etime-stime
    assert eps > etime-stime

do_test(wait_on_nothing)

def running_check(s, ex):
    assert not s.running()
    s.execute_async(niter=5)
    assert s.running()
    time.sleep(ex+eps)
    assert not s.running()

do_test(running_check)

def wait_check(s, ex):
    print __name__, s
    t = time.time()
    s.execute_async(niter=5)
    assert time.time() - t < ex
    s.wait()
    print time.time() - t > ex+eps  # we might be multithreaded
    assert not s.running()

do_test(wait_check)
sys.exit(0)
    

    #tpool()
    #tpool_throw_on_double_execute()
    #tpool_wait_on_nothing()
    #tpool_interrupt()
#else:
#    print "threadpool async execution tests disabled due to lack of hardware concurrency"

#sthreaded()

#print "okay."

