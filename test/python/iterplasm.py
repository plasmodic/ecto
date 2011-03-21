#!/usr/bin/env python

import ecto, buster, makeplasm

plasm = makeplasm.makeplasm()


for pr in plasm.edges:
    n = pr.key()
    e = pr.data()
    print n, " => ", e
    print n.Name(), len(n.inputs), len(n.outputs), len(n.params)
    for i in n.inputs:
        print "in:", i
    for o in n.outputs:
        print "out:", o
        
    for ds in e.downstream:
        print "ds:", ds
        for mod in ds.data():
            print "   mod:", mod
    for us in e.upstream:
        print "us:", us
