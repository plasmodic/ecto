#!/usr/bin/env python
import ecto

debug = True

class Mult(ecto.Module):
    """Mult is documented"""
    def __init__(self,**kwargs):
        ecto.Module.__init__(self,**kwargs)
        
    @staticmethod
    def Params(params):
        params.declare("factor", "multiply input by this", None)
        
    def config(self):
        self.inputs.declare("input","mul", None)
        self.outputs.declare("out", "multed", None)
        
    def process(self):
        f = self.params.factor
        b = self.inputs.input
        self.outputs.out = b * f

class Identity(ecto.Module):
    def __init__(self, **kwargs):
        #setup internal plasm here:
        #this is only called once, so safe to allocate
        self.plasm = ecto.Plasm()
        self.m = Mult()
        self.d = Mult()
        self.plasm.connect(self.m,"out",self.d,"input")
        
        #call init, which will thunk to Params, then to config
        ecto.Module.__init__(self,**kwargs)
        
    @staticmethod
    def Params(params):
        params.declare("factor", "multiply input by this", None)

    def config(self):
        self.inputs.declare("input","mul", None)
        self.outputs.declare("out", "multed", None)
        f = self.params.factor
        self.m.params.factor = f
        self.d.params.factor = 1.0/f
        self.m.config()
        self.d.config()
        if debug:
            ecto.view_plasm(self.plasm)

    def process(self):
        m = self.m
        d = self.d
        plasm = self.plasm
        m.inputs.input = self.inputs.input
        plasm.mark_dirty(m)
        plasm.go(d)
        self.outputs.out = d.outputs.out

def test_python_module():
    mod = Identity(factor = 5.3)
    mod.inputs.input = 10
    mod.process()
    print mod.outputs.out
    assert mod.outputs.out == 10

if __name__ == '__main__':
    test_python_module()
