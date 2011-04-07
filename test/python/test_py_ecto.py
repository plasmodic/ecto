#!/usr/bin/env python
import ecto
import buster

class MyModule(ecto.module):
    def __init__(self):
        ecto.module.__init__(self)
    @staticmethod
    def Params(params):
        params.set("a","a parameter that will change the world", 4)
        params.set("b", "another param.","hello there")
    def Config(self):
        self.outputs.set("y", "i'll give you this", "hello")
    def Process(self):
        a = self.params["a"].get()
        b = self.params["b"].get()
        self.outputs["y"].set(a*b)
def test_my_module():
    mm = ecto.make( MyModule, a=20,b="spam")
    plasm = ecto.Plasm()
    plasm.go(mm)
    print mm.outputs["y"].get()
    assert(mm.outputs["y"].get() == 20*"spam")

if __name__ == '__main__':
    test_my_module()