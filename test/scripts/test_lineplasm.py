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
import ecto.ecto_test as ecto_test
import sys


def test_plasm(Sched, nthreads, niter, n_nodes, incdelay):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate("Gen", step=1.0, start=0.0)
    inc = ecto_test.Increment("Increment 0", delay=incdelay)

    plasm.connect(gen, "out", inc, "in")

    for j in range(n_nodes-1): # one has already been added
        inc_next = ecto_test.Increment("Increment_%u" % (j+1), delay=incdelay)
        plasm.connect(inc, "out", inc_next, "in")
        inc = inc_next

    printer = ecto_test.Printer("Printy")
    plasm.connect(inc, "out", printer, "in")
#
#    o = open('graph.dot', 'w')
#    print >>o, plasm.viz()
#    o.close()
#    print "\n", plasm.viz(), "\n"
    sched = Sched(plasm)
    sched.execute(niter)

    print "RESULT:", inc.outputs.out
    shouldbe = float(n_nodes + niter - 1)
    print "expected:", shouldbe
    assert inc.outputs.out == shouldbe

    #sched.execute(niter)
    #result = inc.outputs.out
    #print "RESULT:", result

    #shouldbe = float(n_nodes + (niter*2 - 1))
    #assert result == shouldbe

    return


if __name__ == '__main__':
    test_plasm(ecto.Scheduler,
               nthreads=int(sys.argv[1]),
               niter=int(sys.argv[2]),
               n_nodes=int(sys.argv[3]),
               incdelay=int(sys.argv[4])
               )
