#!/usr/bin/env python
import sys, ecto, ecto.schedulers
import ecto_test
from util import fail
class MyBlackBox(ecto.BlackBox):
    ''' A simple black box that doesn't really do anything.
    '''
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    def declare_params(self, parameters):
        parameters.declare("fail", "Should i fail or should i go.", False)

    def init_cells(self, parameters):
        self.generate = ecto_test.Generate()
        self.inc = ecto_test.Increment()
        self.printer = ecto_test.Printer()
        self.fail = parameters.fail

    def expose_outputs(self):
        return {
                "out":self.inc["out"]
               }

    def expose_parameters(self):
        return {
                "start":self.generate["start"],
                "step":self.generate["step"],
                }

    def connections(self):
        graph = [
                self.generate["out"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
        if self.fail:
            graph += [ ecto_test.ExceptInConstructor()]
        return graph

def test_bb(options):
    mm = MyBlackBox(start=10, step=3, niter=2)
    plasm = ecto.Plasm()
    plasm.insert(mm)
    options.niter = 5
    run_plasm(options, plasm)
    assert mm.outputs.out == 38
    run_plasm(options, plasm)
    assert mm.outputs.out == 68

def test_bb_fail(options):
    mm = MyBlackBox("MaMaMa", start=10, step=3, fail=True)
    assert 'fail' in  mm.__doc__
    assert mm.name() == 'MaMaMa'
    plasm = ecto.Plasm()
    plasm.insert(mm)
    try:
        run_plasm(options, plasm)
        fail()
    except ecto.CellException, e:
#        print str(e)
        assert "ecto_test::ExceptInConstructor" in str(e)

def test_command_line_args():
    import argparse
    from ecto.opts import cell_options
    parser = argparse.ArgumentParser()
    bb_factory = cell_options(parser, MyBlackBox, 'bb')
    import StringIO
    f = StringIO.StringIO()
    parser.print_help(f)
    for x in f:
        print x

def test_command_line_args2():
    import argparse
    from ecto.opts import cell_options
    parser = argparse.ArgumentParser()
    bb_factory = cell_options(parser, MyBlackBox, 'bb')
    args = parser.parse_args(['--bb_start', '102'])
    mm = bb_factory(args)
    assert mm.params.start == 102

if __name__ == '__main__':
    test_command_line_args()
    test_command_line_args2()
    from ecto.opts import scheduler_options, run_plasm
    import argparse
    parser = argparse.ArgumentParser()
    scheduler_options(parser)
    options = parser.parse_args()
    test_bb(options)
    test_bb_fail(options)
