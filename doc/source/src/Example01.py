#!/usr/bin/python 

import ecto
from ecto_examples import Example01

ecell = Example01(value=17)

p = ecto.Plasm()
p.insert(ecell)
p.execute()

