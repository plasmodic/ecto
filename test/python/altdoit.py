#!/usr/bin/env python

import ecto
from ecto.doc import printModuleDoc
import buster

class MyMod(ecto.module):
    def __init__(self):
        ecto.module.__init__(self)

    def Config(self):
        print "CONFIG!!!!!!"
        self.setIn("numberin", "this number is in", 777)
        self.setOut("numberout", "this number is out", 1313)

    def Process(self):
        print "PROCESS!!!!!"

        value = self.getIn("numberin") * 17
        print "value is", value
        self.put("numberout", value)





print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()

m = MyMod()
print m
m.Config()

printModuleDoc(m)

plasm.markDirty(m)
plasm.go(m)

printModuleDoc(m)

t = """
MyMod:
  inputs: 
   numberIn: 777
  outputs:
   numberOut: 1313
"""

import yaml
d = yaml.load(t)
for k, v in d.iteritems():
    x= init_from_config(MyMod, k , v)
    inputs = v['inputs']
    for k2, v2 in inputs.iteritems():
        x.setIn(label, k2, v2)