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
        fail("sched already running")

    except ecto.EctoException, e:
        print e
        assert "threadpool scheduler already running" in str(e)

    try:
        s.execute_async()
        fail("sched already running")

    except ecto.EctoException, e:
        print e
        assert "threadpool scheduler already running" in str(e)


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
    tpool_throw_on_double_execute()
    tpool_wait_on_nothing()
    tpool_interrupt()
else:
    print "threadpool async execution tests disabled due to lack of hardware concurrency"

sthreaded()

print "okay."

