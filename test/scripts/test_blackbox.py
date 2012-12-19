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
import sys, ecto
import ecto.ecto_test as ecto_test
from util import fail

class MyBlackBox(ecto.BlackBox):
    ''' A simple black box that doesn't really do anything.
    '''

    @classmethod
    def declare_cells(cls, p):
        return {'gen': ecto.BlackBoxCellInfo(ecto_test.Generate, {}, 'all'),
                'inc': ecto.BlackBoxCellInfo(ecto_test.Increment, {}, [ecto.BlackBoxForward('amount','','')])
               }

    @classmethod
    def declare_direct_params(cls, p, **kwargs):
        p.declare("fail", "Should i fail or should i go.", False)

    @classmethod
    def declare_forwarded_io(cls, p):
        return ({},{'inc': [ecto.BlackBoxForward('out', 'value', 'New docs')]})

    def configure(self, p, i, o):
        self.fail = p.at('fail').val
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
    # final value is start + step*(2*5-1)+amount
    assert mm.outputs.value == 39
    run_plasm(options, plasm)
    # final value is start + step*(2*(5+5)-1)+amount
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

class MyBlackBox2(ecto.BlackBox):
    '''
    This BlackBox tests:
    - a BlackBox within a Blacbox
    - some parameters are implicitly declared
    - no declare_direct_params
    - two cells of the same type are part of it
    '''

    @classmethod
    def declare_cells(cls, p):
        return {'gen': ecto.BlackBoxCellInfo(MyBlackBox, {'start':20}, [ecto.BlackBoxForward('step','',''),
                                                                        ecto.BlackBoxForward('amount','amount1','')]),
                'inc': ecto.BlackBoxCellInfo(ecto_test.Increment, {}, [ecto.BlackBoxForward('amount','amount2','')])
               }

    @classmethod
    def declare_forwarded_io(cls, p):
        return ({},{'inc': [ecto.BlackBoxForward('out', 'value', 'New docs')]})

    def configure(self, p, i, o):
        self.printer = ecto_test.Printer()

    def connections(self):
        graph = [
                self.gen["value"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
        return graph

def test_bb2(options):
    # start is going to be ignored as it is set to 20 by default
    mm = MyBlackBox2(start=0, step=3, amount1=10, amount2=50)
    plasm = ecto.Plasm()
    plasm.insert(mm)
    options.niter = 5
    run_plasm(options, plasm)
    # final value is start + step*(5-1)+amount1+amount2
    assert mm.outputs.value == 92
    run_plasm(options, plasm)
    # final value is start + step*(5+5-1)+amount1+amount2
    assert mm.outputs.value == 107

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

    test_bb2(options)
