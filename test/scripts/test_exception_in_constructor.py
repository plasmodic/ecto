#!/usr/bin/env python
import ecto
import ecto_test
import sys, util

exceptor = ecto_test.ExceptInConstructor()

plasm = ecto.Plasm()
plasm.insert(exceptor)

expected_except='''Original Exception: std::logic_error
  What   : I hate life.
  Module : ecto_test::ExceptInConstructor
  in constructor of: ecto_test::ExceptInConstructor
  Module : ecto_test::ExceptInConstructor
  Function: configure'''
try:
    plasm.execute(niter=1)
    util.fail()
except RuntimeError, e:
    print sys.exc_info()
    print str(e)
    assert "what  I hate life." in str(e)
    assert "when  Construction" in str(e)
