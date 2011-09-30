#!/usr/bin/env python
import sys, ecto, ecto.schedulers
import ecto_test

class MyModule(ecto.BlackBox):
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    def init_cells(self, parameters):
        self.generate = ecto_test.Generate()
        self.inc = ecto_test.Increment()
        self.printer = ecto_test.Printer()

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

class MyModule2(ecto.BlackBox):
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    def init_cells(self, parameters):
        self.generate = ecto_test.Generate()
        self.inc = ecto_test.Increment()

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
               ]

#
# verify that blackbox can have same in and out
#
class SameInAndOut(ecto.BlackBox):
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    def init_cells(self, parameters):
        self.passthrough = ecto.Passthrough()
        self.inc = ecto_test.Increment()

    def expose_inputs(self):
        return {
                "same":self.passthrough["in"]
               }
    def expose_outputs(self):
        return {
                "same":self.inc["out"]
               }
    def connections(self):
        return [
                self.passthrough["out"] >> self.inc["in"],
               ]

def test_blackbox():
    plasm = ecto.Plasm()
    mm = MyModule(start=10, step=3)
    inc = ecto_test.Increment()
    print mm.outputs.out
    assert mm.outputs.out == 0
    plasm.connect(mm["out"] >> inc["in"])
    plasm.execute(11)
    print mm.outputs.out
    print mm.parameters.start
    print mm.parameters.step
    print mm.outputs.out
    assert mm.outputs.out == 41 # 10 + 10*3 + 1
    try:
        print mm.inputs.input
        fail("should have thrown")
    except ecto.EctoException, e:
        pass
    #single item in connections list.
    mm = MyModule2(start=10, step=3)

def test_blackbox2():
    plasm = ecto.Plasm()
    sii = SameInAndOut()
    gen = ecto_test.Generate()
    pr = ecto_test.Printer()
    plasm.connect(gen[:] >> sii['same'],
                  sii['same'] >> pr[:])


if __name__ == '__main__':
    test_blackbox()
    test_blackbox2()  # this is currently broken, q.v. #100
