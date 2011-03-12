#!/usr/bin/env python

import ecto
import buster

def printModuleDoc(m):
    print "inputs:"
    for x in m.inputs :
        print "\t",x.data().name(),"type=%s"%x.data().type_name()
        print "\t\t", x.data().doc()
    print "outputs:"
    for x in m.outputs :
        print "\t",x.data().name(),"type=%s"%x.data().type_name()
        print "\t\t", x.data().doc()

g = buster.Generate();
g.Config(17, 3)
printModuleDoc(g)

m = buster.Multiply();
m.Config(2);
printModuleDoc(m)


g.connect("out", m, "in")
g.Process()

print "gout:", g.outputs["out"].value()

m.Process()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()

g.Process()
print "gout:", g.outputs["out"].value()

m.Process()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()

print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()
m2 = buster.Multiply()
m3 = buster.Multiply()
m4 = buster.Multiply()
m4.Config(4)
m2.Config(3)
m3.Config(5)
m.Config(5)
g.Config(17, 3)
try:
    plasm.connect(g, "out", m, "fin")
except RuntimeError,e:
    print "caught a type mismatch"
    print e
plasm.connect(g, "out", m2, "in")
plasm.connect(m2, "out", m3, "in")
plasm.connect(m2, "out", m4, "in")
plasm.markDirty(g)
plasm.go(m)
plasm.go(m4)
print plasm.viz()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()
print "m2out:", m2.outputs["out"].value()
print "m3out:", m3.outputs["out"].value()




