#!/usr/bin/env python
import ecto
import ecto_test
import sys, util

plasm = ecto.Plasm()
exceptor = ecto_test.ExceptInConstructor()
plasm.insert(exceptor)
sched = ecto.schedulers.Singlethreaded(plasm)

try:
    sched.execute(1)
except ecto.CellException, e:
    print "except!\n", e
    assert "what  I hate life." in str(e)
    assert "when  Construction" in str(e)
    assert "type  std::logic_error" in str(e)
    assert "exception_type  CellException" in str(e)
sys.exit(0)
