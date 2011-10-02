#!/usr/bin/env python
from ecto import Constant
from ecto_test import Multiply
from ecto.opts import CellYamlFactory
import yaml

m_factory = CellYamlFactory(Multiply, 'mult')

#write to file
with open('mult.yaml','w') as f:
    m_factory.dump(f)

#to string
print '#some yaml\n',m_factory.dump() 

#read from file
parsed = {}
with open('mult.yaml','r') as f:
    parsed = yaml.load(f)

#create an instance from a parsed yaml file
m = m_factory.load(parsed,cell_name='my_mult_instance')
print m, m.name(), m.params.factor
