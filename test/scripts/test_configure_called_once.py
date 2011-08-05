#!/usr/bin/env python
from ecto_test import ConfigureCalledOnce

cco = ConfigureCalledOnce()

cco.configure()
cco.process()
cco.configure()
cco.configure()
