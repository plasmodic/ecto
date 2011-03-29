#!/usr/bin/env python

import ecto
from ecto.doc import printModuleDoc
import buster

class MyMod(ecto.module):
    def __init__(self):
        ecto.module.__init__(self)
    @staticmethod
    def Params(params):
        print type(params)
        print len(params)
        #params["p1"]
    def Config(self):
        print "CONFIG!!!!!!"
        #self.x = self.getParam("p1")
        #self.inputs.["dafj"]
        self.setIn("numberin", "this number is in", 777)
        self.setOut("numberout", "this number is out", 1313)

    def Process(self):
        print "PROCESS!!!!!"

        value = self.getIn("numberin") * 2
        print "value is", value
        self.put("numberout", value)





print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()

m = MyMod()
MyMod.Params(m.params)
print m
m.Config()
printModuleDoc(m)

plasm.markDirty(m)
plasm.go(m)

printModuleDoc(m)

g = buster.Gather()
g.Params(g.params)
printModuleDoc(g)
g.params["n"].set(3)
g.Config()
printModuleDoc(g)
a = ecto.tendrils()
a.__setitem__("a",ecto.tendril())
a["b"] = ecto.tendril()
print help(a)
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