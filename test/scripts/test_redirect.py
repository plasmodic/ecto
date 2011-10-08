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
import sys


fname = "test_redirect.log"

#FIXME This things hangs.

def make(Schedtype):
    print 'Using :', Schedtype
    plasm = ecto.Plasm()

    gen = ecto_test.Generate("Gen", step=1.0, start=0.0)

    printer = ecto_test.Printer("Printy")
    ecto.log_to_file(fname)
    plasm.connect(gen[:] >> printer[:])
    return Schedtype(plasm)

def verify():
    f = open(fname)
    txt = f.read()
    print ">>>", txt, "<<<"
    assert len(txt.splitlines()) >= 5
    

sched = make(ecto.schedulers.Singlethreaded)
sched.execute(niter=5)
verify()

sched = make(ecto.schedulers.Singlethreaded)
sched.execute_async(niter=5)
sched.wait()
verify()



sched = make(ecto.schedulers.Threadpool)
sched.execute(niter=5)
verify()

sched = make(ecto.schedulers.Threadpool)
sched.execute_async(niter=5)
sched.wait()
verify()



