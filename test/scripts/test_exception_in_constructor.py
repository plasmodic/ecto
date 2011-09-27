#!/usr/bin/env python
import ecto
import ecto_test
import sys, util

exceptor = ecto_test.ExceptInConstructor()

try:
    exceptor.configure()
except RuntimeError, e:
    print "except!\n", e


sys.exit(0)
