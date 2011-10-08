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
#
#  This script really just generates load, not actually used in testing
#
import ecto
import ecto_test
import sys

def build_crunchgraph(nlevels, ncalls):
    
    plasm = ecto.Plasm()
    prevlevel = [ecto_test.Add("Adder 0_%u" % x) for x in range(2**(nlevels-1))]
    for adder in prevlevel:
        plasm.connect(ecto_test.Uniform01(ncalls=ncalls)["out"] >> adder["left"],
                      ecto_test.Uniform01(ncalls=ncalls)["out"] >> adder["right"])

    print "prev has", len(prevlevel)

    for k in range(nlevels-2, -1, -1):
        print "****** k=", k, " ***********"
        thislevel = [ecto_test.Add("Adder %u_%u" % (k, x)) for x in range(2**k)]
        print "prevlevel=", prevlevel
        print "thislevel=", thislevel
        index = 0
        print "for...", range(2**k)
        for r in range(2**k):
            print "prev[%u] => cur[%u]" % (index, r)
            plasm.connect(prevlevel[index]["out"] >> thislevel[r]["left"])
            index += 1
            print "prev[%u] => cur[%u]" % (index, r)
            plasm.connect(prevlevel[index]["out"]>>thislevel[r]["right"])
            index += 1
        prevlevel = thislevel

    assert len(prevlevel) == 1
    final_adder = prevlevel[0]
    printer = ecto_test.Printer("printy!")

    return (plasm, final_adder)

def test_plasm(nlevels, ncalls, niter):
    (plasm, outnode) = build_crunchgraph(nlevels, ncalls)

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
    print "RESULT:", outnode.outputs.out

if __name__ == '__main__':
    test_plasm(nlevels=6, ncalls=10000, niter=6000)





