#!/usr/bin/env python
import ecto, util

# if this catching works for one, it will work for them all..

try:
    raise ecto.ValueNone("erp")
except:
    print "ok!"

try:
    raise ecto.ValueNone("erp")
except Exception:
    print "ok!"

try:
    raise ecto.ValueNone("erp")
except RuntimeError:
    print "ok!"

try:
    raise ecto.ValueNone("erp")
except ecto.TypeMismatch:
    util.fail()
except ecto.ValueRequired:
    util.fail()
except ecto.ValueNone:
    print "ok!"





