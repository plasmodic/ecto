#!/usr/bin/env python
import ecto
import buster
from ecto import module 

class MyModule(ecto.module):
    def __init__(self, *args, **kwargs):
        ecto.module.__init__(self, **kwargs)
        
    @staticmethod
    def Params(params):
        params.declare("text", "a param.","hello there")
        
    def Config(self):
        self.text = self.params.text
        self.inputs.declare("input","aye", 2)
        self.outputs.declare("out", "i'll give you this", "hello")
        
    def Process(self):
        c = int(self.inputs.input)
        self.outputs.out = c*self.text
        print MyModule.__name__

class Mult(ecto.module):
    """Mult is documented"""
    def __init__(self, *args, **kwargs):
        print "HERE"
        ecto.module.__init__(self, **kwargs)
        print "THERE"
        
    @staticmethod
    def Params(params):
        params.declare("factor", "multiply input by this", 4)
        
    def Config(self):
        self.inputs.declare("input","mul", 2)
        self.outputs.declare("out", "multed",8)
        
    def Process(self):
        a = self.params.factor
        b = self.inputs.input
        self.outputs.out = b * a
        
def test_tendril_getattr():
    gen = buster.Generate(start=2, step=3)
    print "OUTPUTS:::", gen.outputs.out
    assert gen.outputs.keys() == ['out']
    assert gen.outputs.out == -1
    print gen.inputs.keys()
    assert gen.inputs.keys() == []

    gen.Process()

    print gen.outputs.out
    assert gen.outputs.out == 2.0
    
    try:
        print gen.outputs.nonexistent
        assert "that should have thrown"
    except RuntimeError, e:
        assert 'does not exist!' in str(e)

def test_my_module():
    t = ecto.tendrils()
    print t

    mm = MyModule(text="spam")
    mm.Config()
    mul = Mult(factor=4)
    mul.Config()
    printer = buster.Printer()
    printer.Config()
    print printer
    gen = buster.Generate(start=2, step=3)
    gen.Config()
    plasm = ecto.Plasm()
    print str(gen.outputs)
    print str(mul.inputs)
    plasm.connect(gen,"out",mul,"input")
    plasm.connect(mul,"out",mm, "input")
    plasm.connect(mm,"out",printer,"str")
    print plasm.go
    plasm.go(printer)
    plasm.mark_dirty(gen)
    plasm.go(printer)
    plasm.mark_dirty(gen)
    plasm.go(printer)
    ecto.print_module_doc(mul)
    #ecto.view_plasm(plasm)
    print "it is:", mm.outputs.out
    assert(mm.outputs.out == "spamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspam")

if __name__ == '__main__':
    test_my_module()
