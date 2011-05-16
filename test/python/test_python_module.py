#!/usr/bin/env python
import ecto
import buster

class MyModule(ecto.Module):
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)
        
    @staticmethod
    def Initialize(params):
        params.declare("text", "a param.","hello there")

    def configure(self):
        self.text = self.params.text
        self.inputs.declare("input","aye", 2)
        self.outputs.declare("out", "i'll give you this", "hello")
        
    def process(self):
        c = int(self.inputs.input)
        self.outputs.out = c*self.text

def test_python_module():
    mod = MyModule(text="spam")
    assert mod.text == "spam"
    assert mod.params.text == "spam"
    mod.process()
    assert mod.outputs.out == "spam"*2
    assert mod.outputs["out"].val == "spam"*2

def test_python_module_plasm():
    mod = MyModule(text="spam")
    g = buster.Generate(start = 1 , step =1)
    plasm = ecto.Plasm()
    plasm.connect(g,"out",mod,"input")
    plasm.set_input(g)
    plasm.set_output(mod)
    for i in range(1,5):
        plasm.execute()
        assert g.outputs.out == i
        print mod.outputs.out
        assert mod.outputs.out == "spam"*i
    plasm.execute()
    assert g.outputs.out == 5
    print mod.outputs.out
    assert mod.outputs.out == "spam"*5
    
if __name__ == '__main__':
    test_python_module()
    test_python_module_plasm()
