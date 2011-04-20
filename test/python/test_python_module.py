#!/usr/bin/env python
import ecto

class MyModule(ecto.Module):
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)
        
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

def test_python_module():
    mod = MyModule(text="spam")
    assert mod.text == "spam"
    assert mod.params.text == "spam"
    mod.Process()
    assert mod.outputs.out == "spam"*2
    assert mod.outputs["out"].val == "spam"*2

if __name__ == '__main__':
    test_python_module()
