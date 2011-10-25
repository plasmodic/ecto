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
    startstop = ecto_test.StartStopCounter()

    plasm.connect(ping[:] >> startstop[:])

    return (plasm, startstop)

def do_test(fn):
    def impl(Sched):
        print "*"*80, "\n", fn.__name__, Sched.__name__
        (p, ss) = makeplasm()
        s = Sched(p)
        fn(s, ss)
    map(impl, [ecto.schedulers.Multithreaded])#ecto.test.schedulers)


def synctwice(s, ss):

    print "*"*80
    print "\n"*5
    assert ss.outputs.nstart == 0
    assert ss.outputs.nstop == 0
    assert ss.outputs.nconfigure == 0
    assert ss.outputs.nprocess == 0

    s.execute(niter=5)

    print "NSTART=", ss.outputs.nstart
    assert ss.outputs.nstart == 1
    print "NSTOP=", ss.outputs.nstop
    assert ss.outputs.nstop == 1
    assert ss.outputs.nconfigure == 1
    print "NPROCESS=", ss.outputs.nprocess
    assert ss.outputs.nprocess == 5

    s.execute(niter=5)

    print "NSTART=", ss.outputs.nstart
    assert ss.outputs.nstart == 2
    print "NSTOP=", ss.outputs.nstop
    assert ss.outputs.nstop == 2
    assert ss.outputs.nconfigure == 1
    print "NPROCESS=", ss.outputs.nprocess
    assert ss.outputs.nprocess == 10

    s.execute_async(niter=5)
    while s.running():
        time.sleep(0.1)

    print "NSTART=", ss.outputs.nstart
    assert ss.outputs.nstart == 3
    print "NSTOP=", ss.outputs.nstop
    assert ss.outputs.nstop == 3
    assert ss.outputs.nconfigure == 1
    print "NPROCESS=", ss.outputs.nprocess
    assert ss.outputs.nprocess == 15

    s.wait()
    s.execute_async()
    time.sleep(1.0)
    s.stop()
    s.wait()
    print s.stats(), "\n"*5
    print "NSTART=", ss.outputs.nstart
    assert ss.outputs.nstart == 4
    assert ss.outputs.nconfigure == 1


    assert ss.outputs.nstop == 4
    assert ss.outputs.nconfigure == 1
    print "NPROCESS=", ss.outputs.nprocess
    assert ss.outputs.nprocess > 15

for j in range(ecto.test.iterations):
    do_test(synctwice)


def things_not_too_slow(s, ss):

    s.execute_async()
    time.sleep(1.0)
    s.stop()
    s.wait()
    print s.stats()
    s.execute_async()
    time.sleep(1.0)
    s.stop()
    s.wait()
    print s.stats()


do_test(things_not_too_slow)
