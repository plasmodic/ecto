#!/usr/bin/env python

import pprint, sys
import ecto

# pprint.pprint(ecto.__dict__)

andcell = ecto.And("Andcell", ninput=17)

print andcell
print andcell.type()
print andcell.inputs.keys()
print andcell.outputs.keys()
print andcell.params.keys()

sys.exit(0)
print andcell.__doc__
print ecto.And.__doc__
print andcell.type()
assert andcell.type() == 'ecto::And'
pprint.pprint(andcell.__dict__)
pprint.pprint(ecto.And.__dict__)
print andcell.inputs
print andcell.inputs.in1, andcell.inputs.in2

a2 = ecto.And()
print a2.inputs
print a2.inputs.in1, a2.inputs.in2
a2.inputs.in1 = False
print a2.inputs.in1, a2.inputs.in2
print "TT:", andcell.inputs.in1, andcell.inputs.in2
print help(andcell)
