#!/usr/bin/env python
import ecto
import buster
debug = True

class Mult(ecto.Module):
    """Mult is documented"""
    def __init__(self,**kwargs):
        ecto.Module.__init__(self,**kwargs)
        
    @staticmethod
    def Initialize(params):
        params.declare("factor", "multiply input by this", None)
        
    def configure(self):
        self.inputs.declare("input","mul", None)
        self.outputs.declare("out", "multed", None)
        
    def process(self):
        f = self.params.factor
        b = self.inputs.input
        self.outputs.out = b * f
       
def identity(factor):
    plasm = ecto.Plasm()
    m = Mult(factor = factor)
    d = Mult(factor = 1/factor)
    plasm.connect(m,"out",d,"input")
    return (plasm,(m,),(d,))


def test_python_module():
    plasm = identity(5.3)
    mod.inputs.input = 10
    mod.process()
    print mod.outputs.out
    assert mod.outputs.out == 10

if __name__ == '__main__':
    test_python_module()
