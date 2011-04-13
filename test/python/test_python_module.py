#!/usr/bin/env python
import ecto
class Mult(ecto.module):
    """Mult is documented"""
    def __init__(self,*args, **kwargs):
        ecto.module.__init__(self, **kwargs)
        
    @staticmethod
    def Params(params):
        params.set("factor", "multiply input by this", None)
        
    def Config(self):
        self.inputs.set("input","mul", None)
        self.outputs.set("out", "multed", None)
        
    def Process(self):
        f = self.params.factor
        b = self.inputs.input
        self.outputs.out = b * f

class Identity(ecto.module):
    def __init__(self, *args, **kwargs):
        self.plasm = plasm = ecto.Plasm()
        self.m = m = Mult()
        self.d = d = Mult()
        plasm.connect(m,"out",d,"input")
        ecto.module.__init__(self, **kwargs)
        
    @staticmethod
    def Params(params):
        params.set("factor", "multiply input by this", None)

    def Config(self):
        f = self.params.factor
        self.m.params.factor = f
        self.d.params.factor = 1.0/f
        self.m.Config()
        self.d.Config()
        self.inputs.set("input","mul", None)
        self.outputs.set("out", "multed", None)
            
    def Process(self):
        m = self.m
        d = self.d
        m.inputs["input"] = self.inputs["input"]
        self.plasm.mark_dirty(m)
        self.plasm.go(d)
        self.outputs["out"] = d.outputs["out"]

def test_python_module():
    mod = Identity(factor = 5.3)
    mod.inputs.input = 10
    mod.Process()
    print mod.outputs.out

if __name__ == '__main__':
    test_python_module()
