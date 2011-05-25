#!/usr/bin/env python
import ecto
import buster

class MyModule(ecto.Module):
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)
        
    def declare_params(self, params):
        params.declare("text", "a param.","hello there")

    def declare_io(self,params, inputs, outputs):
        self.text = params.text
        inputs.declare("input","aye", 2)
        outputs.declare("out", "i'll give you this", "hello")
        
    def process(self,inputs, outputs):
        c = int(inputs.input)
        outputs.out = c * self.text

def test_python_module():
    mod = MyModule(text="spam")
    assert mod.text == "spam"
    assert mod.params.text == "spam"
    mod.process(mod.inputs,mod.outputs)
    assert mod.outputs.out == "spam"*2
    assert mod.outputs["out"].val == "spam"*2

def test_python_module_plasm():
    mod = MyModule(text="spam")
    g = buster.Generate(start = 1 , step =1)
    plasm = ecto.Plasm()
    plasm.connect(g,"out",mod,"input")
    #print plasm.viz()
    for i in range(1,5):
        plasm.execute()
        assert g.outputs.out == i
        assert mod.outputs.out == "spam"*i
    plasm.execute()
    assert g.outputs.out == 5
    assert mod.outputs.out == "spam"*5
    
if __name__ == '__main__':
    test_python_module()
    test_python_module_plasm()
