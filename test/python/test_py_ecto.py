#!/usr/bin/env python
import ecto
import buster

class MyModule(ecto.module):
    def __init__(self):
        ecto.module.__init__(self)
        
    @staticmethod
    def Params(params):
        params.set("text", "a param.","hello there")
        
    def Config(self):
        self.text = self.params["text"].val
        self.inputs.set("in","aye", 2)
        self.outputs.set("out", "i'll give you this", "hello")
        
    def Process(self):
        c = int(self.inputs["in"].val)
        self.outputs["out"].val = c*self.text
        print MyModule.__name__

class Mult(ecto.module):
    """Mult is documented"""
    def __init__(self):
        ecto.module.__init__(self)
        
    @staticmethod
    def Params(params):
        params.set("factor","multiply input by this", 4)
        
    def Config(self):
        self.inputs.set("in","mul", 2)
        self.outputs.set("out", "multed",8)
        
    def Process(self):
        a = self.params["factor"].val
        b = self.inputs["in"].val
        self.outputs["out"].val = b * a
        
def test_my_module():
    mm = ecto.make( MyModule, text="spam")
    mul = ecto.make( Mult, factor=4)
    printer = ecto.make(buster.Printer)
    gen = ecto.make(buster.Generate,start=2,step=3)
    plasm = ecto.Plasm()
    plasm.connect(gen,"out",mul,"in")
    plasm.connect(mul,"out",mm, "in")
    plasm.connect(mm,"out",printer,"str")
    plasm.go(printer)
    plasm.mark_dirty(gen)
    plasm.go(printer)
    plasm.mark_dirty(gen)
    plasm.go(printer)
    ecto.print_module_doc(mul)
    #ecto.view_plasm(plasm)
    assert(mm.outputs["out"].val == "spamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspamspam")

if __name__ == '__main__':
    test_my_module()