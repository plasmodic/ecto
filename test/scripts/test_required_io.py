#!/usr/bin/env python
import ecto
import ecto_test
import sys


print ecto_test.RequiredIO.__doc__
#test
assert "out [double] REQUIRED" in ecto_test.RequiredIO.__doc__
assert "in [double] REQUIRED" in ecto_test.RequiredIO.__doc__

gen = ecto_test.Generate()
req = ecto_test.RequiredIO()
out = ecto_test.Printer(print_type="double")

try:
    p = ecto.Plasm()
    p.connect(gen[:] >> req[:])
    print "checking..."
    p.check()
    print "checked."
except Exception, e:
    assert str(e) == "in module ecto_test::RequiredIO, output port 'out' is required but not connected"
    print e, '(threw as expected)'

try:
    p = ecto.Plasm()
    p.connect(req[:] >> out [:])
    p.check()
except Exception, e:
    assert str(e) == "in module ecto_test::RequiredIO, input port 'in' is required but not connected"
    print e, '(threw as expected)'






