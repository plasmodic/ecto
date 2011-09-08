#!/usr/bin/env python
import ecto, ecto_test, sys, util


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
    util.fail()
except Exception, e:
    print str(e)
    assert "NotConnected" in str(e)
    print e, '(threw as expected)'

try:
    p = ecto.Plasm()
    p.connect(req[:] >> out [:])
    p.check()
    util.fail()
except Exception, e:
    print str(e)
    assert "NotConnected" in str(e)
    print e, '(threw as expected)'






