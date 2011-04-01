#!/usr/bin/env python

import ecto
from ecto.doc import printModuleDoc, graphviz
import buster

print "#################\nconfig test\n#################"
s = ecto.make(buster.Scatter)
ecto.config(s, n=5)
printModuleDoc(s)

g = ecto.make(buster.Gather)
ecto.config(g, n=5)
printModuleDoc(g)

m = ecto.make(buster.Multiply)
printModuleDoc(m)

class SG(ecto.module):

    def __init__(self):
        ecto.module.__init__(self)
        self.s = ecto.make(buster.Scatter)
        self.g = ecto.make(buster.Gather)

    @staticmethod
    def Params(params):
        buster.Scatter.Params(params)

    def Config(self):
        plasm = ecto.Plasm()
        s = self.s
        g = self.g
        params = self.params
        s.params["n"].set(params["n"].get())
        s.params["x"].set(params["x"].get())
        g.params["n"].set(params["n"].get())
        ecto.config(s)
        ecto.config(g)
        self.outputs = g.outputs
        for f, t in zip(ecto.keys(s.outputs), ecto.keys(g.inputs)):
            plasm.connect(s, f, g, t)
        self.plasm = plasm

    def Process(self):
        self.plasm.go(self.g)


print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()

for f, t in zip(ecto.keys(s.outputs), ecto.keys(g.inputs)):
    plasm.connect(s, f, g, t)



plasm.go(g)
print g.outputs["out"].get()
sg = ecto.make(SG)
printModuleDoc(sg)
sg.p.x.set(10)
sg.p.n.set(50)
ecto.config(sg)
#ecto.config(sg, x=10,n=50)
plasm.markDirty(sg)
plasm.go(sg)
print sg.outputs["out"].get()

plasm.connect(sg, "out", m, "in")
plasm.go(m)
print m.o.out.get()
print sg.p.__dict__
print graphviz(plasm)
#t = """
#MyMod:
#  inputs: 
#   numberIn: 777
#  outputs:
#   numberOut: 1313
#"""
#
#import yaml
#d = yaml.load(t)
#for k, v in d.iteritems():
#    x= init_from_config(MyMod, k , v)
#    inputs = v['inputs']
#    for k2, v2 in inputs.iteritems():
#        x.setIn(label, k2, v2)
