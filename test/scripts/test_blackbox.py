#!/usr/bin/env python
import ecto, ecto.schedulers
import ecto_test

class BlackBox(ecto.Module):
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)
        self.generate = ecto_test.Generate.inspect(None,None)
        
    @staticmethod
    def declare_params(params):
        params.declare("text", "a param.","hello there")
        generate = ecto_test.Generate.inspect(None,None)
        params = generate.par

    @staticmethod
    def declare_io(params, inputs, outputs):
        inputs.declare("input","aye", 2)
        outputs.declare("out", "i'll give you this", "hello")
    
    def configure(self,params):
        self.text = params.text

    def process(self,inputs, outputs):
        c = int(inputs.input)
        outputs.out = c * self.text

def black_box():
    plasm = ecto.Plasm()
    x =
        
def make_plasm(plasm):
    generate = ecto_test.Generate(start=1, step=3.0)
    inc = ecto_test.Increment()
    plasm.connect(generate, "out", inc, "in")
    plasm.inc = inc
    return plasm

def test_blackbox():
    plasm = make_plasm(ecto.Plasm())
    sched = ecto.schedulers.Singlethreaded(plasm)
    
    for x in range(0, 10):
        sched.execute()
        
    print plasm.inc.outputs.out
    
if __name__ == '__main__':
    test_blackbox()
