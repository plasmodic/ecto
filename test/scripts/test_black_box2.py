#!/usr/bin/env python
import sys, ecto, ecto.schedulers
import ecto_test

class MyBlackBox(ecto.BlackBox):
    def __init__(self, start, step, fail=False):
        ecto.BlackBox.__init__(self)
        self.generate = ecto_test.Generate(start=start, step=step)
        self.inc = ecto_test.Increment()
        self.printer = ecto_test.Printer()
        self.fail = fail

    def expose_outputs(self):
        return {
                "out":self.inc["out"]
               }
    def expose_parameters(self):
        return {
                "start":self.generate["start"],
                "step":self.generate["step"]
                }
    def connections(self):
        return [
                self.generate["out"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
    def cell(self):
        plasm = ecto.Plasm()
        plasm.connect(self.connections())
        if self.fail:
            plasm.insert(ecto_test.ExceptInConstructor())
        return ecto.create_black_box(plasm, niter=2, parameters=self.expose_parameters(), outputs=self.expose_outputs())

def test_bb(options):
    mm = MyBlackBox(start=10, step=3).cell()
    plasm = ecto.Plasm()
    plasm.insert(mm)
    options.niter = 5
    run_plasm(options, plasm)
    assert mm.outputs.out == 38
    run_plasm(options, plasm)
    assert mm.outputs.out == 68

def test_bb_fail(options):
    mm = MyBlackBox(start=10, step=3,fail=True).cell()
    plasm = ecto.Plasm()
    plasm.insert(mm)
    try:
        run_plasm(options, plasm)
    except ecto.CellException, e:
        print str(e)
        assert "ecto_test::ExceptInConstructor" in str(e)

if __name__ == '__main__':
    from ecto.opts import scheduler_options, run_plasm
    import argparse
    parser = argparse.ArgumentParser()
    scheduler_options(parser)
    options = parser.parse_args()
    test_bb(options)
    test_bb_fail(options)
