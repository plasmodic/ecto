#!/usr/bin/env python
import ecto, ecto_test, sys, util

try:
    gen = ecto_test.Generate(step='Foobar')
    util.fail()
except ecto.FailedFromPythonConversion, e:
    print "caught: ", e
    print "which is exactly what we expected.  Wunderbar."
except:
    util.fail()
