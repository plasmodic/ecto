#!/usr/bin/env python
import ecto
import buster
from ecto import module 

class Mult(ecto.module):
    """Mult is documented"""
    def __init__(self, *args, **kwargs):
        print "HERE"
        ecto.module.__init__(self, **kwargs)
        print "THERE"
        
    @staticmethod
    def Params(params):
        params.set("factor", "multiply input by this", 4)
        
    def Config(self):
        self.inputs.set("input","mul", 2)
        self.outputs.set("out", "multed",8)
        
    def Process(self):
        a = self.params.factor
        b = self.inputs.input
        self.outputs.out = b * a
        
class Compound(ecto.module):
    def __init__(self, *args, **kwargs):
        ecto.module.__init__(self, **kwargs)

    @staticmethod
    def Params(params):
        params.set("plasm", "subgraph", None)
        params.set("inputnames", "input names", None)
        params.set("otputnames", "outputs names", None)

    def Config(self):
        for i in self.params.inputnames:
            print "input", i
        for o in self.params.outputnames:
            print "output", o

    def Process(self):
        pass
        # get inputs from inputs
        # put them in to the plasm
        # plasm.go(last...

#subplasm = make_some_plasm()

#mod = Compound(plasm=subplasm, inputs=['foo_in', 'bar_in'], outputs=['foo_out', 'bar_out'])


def test_compound():
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
        assert str(e) == 'nonexistent does not exist!'

if __name__ == '__main__':
    test_my_module()
