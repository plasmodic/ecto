#!/usr/bin/env python
import ecto

class MyModule(ecto.module):
    def __init__(self, *args, **kwargs):
        ecto.module.__init__(self, **kwargs)
        
    @staticmethod
    def Params(params):
        print "setting params!"
        params.set("text", "a param.","hello there")
        print "params=", params

    def Config(self):
        self.text = self.params["text"].val
        self.inputs.set("in","aye", 2)
        self.outputs.set("out", "i'll give you this", "hello")
        
    def Process(self):
        c = int(self.inputs["in"].val)
        self.outputs["out"].val = c*self.text
        print MyModule.__name__

def test_python_module():
    # t = ecto.tendrils()
    # print "t=", t
    # print MyModule.Params
    # MyModule.Params(t)
    # print t
    print MyModule
    mod = MyModule(text="spam")
    mod.Config()

if __name__ == '__main__':
    test_python_module()
