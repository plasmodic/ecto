#!/usr/bin/env python
import ecto, ecto.schedulers
import ecto_test
           
class MyModule(ecto.BlackBox):
    def __init__(self, start, step):
        ecto.BlackBox.__init__(self)
        self.generate = ecto_test.Generate(start=start, step=step)
        self.inc = ecto_test.Increment()
    def _outputs(self):
        return {"out":self.inc["out"]}
    def _parameters(self):
        return {
                "start":self.generate["start"],
                "step":self.generate["step"]
                }
    def connections(self):
        return (self.generate["out"] >> self.inc["in"])
    
def test_blackbox():
    mm = MyModule(start=10, step=3)
    inc = ecto_test.Increment()
    plasm = ecto.Plasm()
    print mm.outputs.out
    assert mm.outputs.out == 0
    plasm.connect(mm.connections())
    print mm["out", "out"]
    plasm.connect(mm["out"] >> inc["in"])    
    plasm.execute(11)
    print mm.outputs.out
    print mm.parameters.start
    print mm.parameters.step
    assert mm.outputs.out == 41 # 10 + 10*3 + 1
    try:
        print mm.inputs.input
        assert False, "Should have thrown, input does not exist!!!"
    except RuntimeError, e:
        print e
    
if __name__ == '__main__':
    test_blackbox()
