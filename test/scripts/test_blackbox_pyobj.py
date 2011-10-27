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
import sys, ecto, ecto.schedulers
import ecto_test
from util import fail

class MyBlackBox(ecto.BlackBox):
    ''' A simple black box that doesn't really do anything.
    '''
    gen = ecto_test.Generate
    inc = ecto_test.Increment

    def declare_params(self, p):
        p.declare("fail", "Should i fail or should i go.", False)
        p.forward("amount", cell_name='inc')
        p.forward_all(cell_name='gen') #carte blanche forward all of the parameters.

    def declare_io(self, p, i, o):
        o.forward('value', cell_name='inc', cell_key='out', doc='New docs')

    def configure(self, p, i, o):
        self.fail = p.fail
        self.gen = self.gen() #custom overriding can occur here.
        self.inc = self.inc()
        self.printer = ecto_test.Printer()

    def connections(self):
        graph = [
                self.gen["out"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
        #something with an bp::object in the params
        graph += [ ecto_test.SleepPyObjectAbuser(list_o_sleeps=[0.1,0.01]) ]
        return graph

def test_bb(options):
    mm = MyBlackBox(start=10, step=3, amount=2, niter=2)
    plasm = ecto.Plasm()
    plasm.insert(mm)
    options.niter = 5
    run_plasm(options, plasm)
    assert mm.outputs.value == 39
    run_plasm(options, plasm)
    assert mm.outputs.value == 69


if __name__ == '__main__':
    from ecto.opts import scheduler_options, run_plasm
    import argparse
    parser = argparse.ArgumentParser()
    scheduler_options(parser)
    options = parser.parse_args()
    test_bb(options)

