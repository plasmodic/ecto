#!/usr/bin/env python

import pprint
import ecto

pprint.pprint(ecto.__dict__)

andcell = ecto.And()
print andcell
print andcell.__doc__
print ecto.And.__doc__
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
