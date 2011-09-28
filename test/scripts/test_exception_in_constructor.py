#!/usr/bin/env python
import ecto
import ecto_test
import sys, util

exceptor = ecto_test.ExceptInConstructor()

try:
    exceptor.configure()
except ecto.CellException, e:
    print "except!\n", e
    assert "what  I hate life." in str(e)
    assert "when  Construction" in str(e)
    assert "type  std::logic_error" in str(e)
    assert "exception_type  CellException" in str(e)

sys.exit(0)
