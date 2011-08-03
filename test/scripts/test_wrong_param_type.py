#!/usr/bin/env python
import ecto
import ecto_test
import sys

try:
    gen = ecto_test.Generate(step='Foobar')
except Exception, e:
    print e
    assert str(e) == '''Could not convert python object to type : double
Parameter: step
Cell: ecto_test::Generate<double>'''
