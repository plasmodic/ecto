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

    gen = ecto_test.Generate("GENERATE", step=1.0, start=1.0)
    noncurr = ecto_test.DontCallMeFromTwoThreads("ALPHA", strand=s1)
    plasm.connect(gen, "out", noncurr, "in")

    for k in range(nlevels):
        next = ecto_test.DontCallMeFromTwoThreads("BETA_%d" % k, strand=s1)
        plasm.connect(noncurr, "out", next, "in")
        noncurr = next

    printer = ecto_test.Printer("PRINTER")
    plasm.connect(noncurr, "out", printer, "in")

    sched = SchedType(plasm)
    print "sched=", sched
    execfn(sched)

    result = noncurr.outputs.out
    print "result=", result
    assert(result == expect)
#    execfn(sched)
#    result = noncurr.outputs.out
#    print "result=", result

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

    sched = ecto.schedulers.Multithreaded(plasm)
    try:
        print "about to execute... this should throw"
        sched.execute(nthreads=4, niter=4)
        util.fail()
    except RuntimeError, e:
        print "good, python caught error", e

#shouldfail()
#print "shouldfail passed"

#test_implicit_strands(4, ecto.schedulers.Multithreaded, lambda s: s.execute(nthreads=4, niter=4), expect=4.0)
#test_implicit_strands(4, ecto.schedulers.Singlethreaded, lambda s: s.execute(niter=4), expect=4.0)

#test_strands(4, ecto.schedulers.Singlethreaded, lambda s: s.execute(niter=4), expect=4.0)
test_strands(4, ecto.schedulers.Multithreaded, lambda s: s.execute(nthreads=4, niter=4), expect=4.0)



