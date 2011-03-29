#!/usr/bin/env python

import ecto
#from ecto.doc import printModuleDoc
import buster

scatter = buster.Scatter()
scatter.Config()

idx = buster.Indexer()
idx.Config()

gather = buster.Gather()
gather.Config(10)

print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()
for x in range(0,10):
    idx = buster.Indexer()
    idx.Config(x)
    plasm.connect(scatter,"out",idx,"in")
    plasm.connect(idx,"out",gather,"in_%04d"%x)
plasm.go(gather)
print "scatter out:", scatter.outputs["out"].value()
print "gather out:", gather.outputs["out"].value()

print "###################\n Plasm Introspection\n###################"

for x in plasm.edges:
    print x.key()
    print x.data()
    edge = x.data()
    for ds in edge.downstream:
        print ds.key()
        for m in ds.data():
            print m
    for up in edge.upstream:
        print up




