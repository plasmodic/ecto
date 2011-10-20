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
    #You must have class variables for any cells which you would like to
    #forward inputs,outputs,params
    #The names of these class variables are assumed to stay consistent
    #and are used when forward declaring.
    #These class level variables are type names, and should be transformed into
    #instances in the configure method.
    gen = ecto_test.Generate
    inc = ecto_test.Increment

    def declare_params(self, p):
        p.declare("fail", "Should i fail or should i go.", False)
        p.forward("amount", cell_name='inc')
        p.forward_all(cell_name='gen') #carte blanche forward all of the parameters.

    def declare_io(self, p, i, o):
        #forward one element
        o.forward('value', cell_name='inc', cell_key='out', doc='New docs')

    def configure(self, p, i, o):
        self.fail = p.fail
        #You must transform instance versions of the class cell types in this
        #function.
        self.gen = self.gen() #custom overriding can occur here.
        self.inc = self.inc()
        self.printer = ecto_test.Printer()

    def connections(self):
        graph = [
                self.gen["out"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
        if self.fail:
            graph += [ ecto_test.ExceptInConstructor() ]
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

def test_bb_fail(options):
    mm = MyBlackBox("MaMaMa", start=10, step=3, fail=True)
    print mm.__doc__
    assert 'fail' in  mm.__doc__
    assert mm.name() == 'MaMaMa'
    plasm = ecto.Plasm()
    plasm.insert(mm)
    try:
        run_plasm(options, plasm)
        fail()
    except ecto.CellException, e:
        print "Good:"
        print str(e)
        assert "I hate life" in str(e)

def test_command_line_args():
    import argparse
    from ecto.opts import cell_options
    parser = argparse.ArgumentParser()
    bb_factory = cell_options(parser, MyBlackBox, 'bb')
    parser.print_help()

def test_command_line_args2():
    import argparse
    from ecto.opts import cell_options, CellYamlFactory
    import yaml
    parser = argparse.ArgumentParser()
    bb_factory = cell_options(parser, MyBlackBox, 'bb')
    args = parser.parse_args(['--bb_start', '102'])
    mm = bb_factory(args)
    assert mm.params.start == 102

def test_yaml():
    from ecto.opts import CellYamlFactory
    import yaml
    bb_yaml = CellYamlFactory(MyBlackBox(start=54), 'bb')
    bb_yaml.dump(sys.stdout)
    mm = bb_yaml.load(yaml.load(bb_yaml.dump()), 'bb')
    print mm.params.start
    assert mm.params.start == 54

if __name__ == '__main__':
    test_command_line_args()
    test_command_line_args2()
    test_yaml()
    from ecto.opts import scheduler_options, run_plasm
    import argparse
    parser = argparse.ArgumentParser()
    scheduler_options(parser)
    options = parser.parse_args()
    test_bb(options)
    test_bb_fail(options)

