#!/bin/python
import ecto
from ecto.doc import printModuleDoc
import orb

plasm = ecto.Plasm()

pyramid = orb.Pyramid()
pyramid.Config(5,1.2,1)
printModuleDoc(pyramid)

FAST = orb.FAST()
printModuleDoc(FAST)

Harris = orb.Harris()
printModuleDoc(Harris)