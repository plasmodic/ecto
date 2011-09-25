#!/usr/bin/env python

import ecto, sys, util
from ecto_test import *

p = ecto.Plasm()

emitters = [Emit_bool, Emit_int, Emit_float, Emit_Struct, Emit_string]
acceptors = [Accept_bool, Accept_int, Accept_float, Accept_Struct, Accept_string]

def do(E, A):
    p = ecto.Plasm()
    e = E()
    a = A()
    ltype = E.__name__.split('_')[1]
    rtype = A.__name__.split('_')[1]
    if ltype == rtype:
        p.connect(e[:] >> a[:])
        return

    def throw():
        p.connect(e[:] >> a[:])
        raise "should have thrown"

    print "connecting, should throw..."
    try:
        throw()
        util.fail()
    except ecto.ValueNone, ex:
        util.fail()
    except ecto.ValueRequired, ex:
        util.fail()
    except ecto.EctoException, ex:
        print "CAUGHT! ok!", ex
        print sys.exc_info()

    try:
        throw()
        util.fail()
    except ecto.TypeMismatch, ex:
        print "CAUGHT! ok!", ex
        print sys.exc_info()

    try:
        throw()
        util.fail()
    except RuntimeError, ex:
        print "okay!", ex
        print sys.exc_info()

def valnone(A):
    p = ecto.Plasm()
    e = Emit_none()
    a = A()
    # doesn't throw yet...
    p.connect(e[:] >> a[:])
    try:
        p.execute(niter=1)
    except ecto.ValueNone, va:
        print "yeah, got ValueNone error"

def typeconv(E):
    p = ecto.Plasm()
    e = E()
    a = Accept_none()
    p.connect(e[:] >> a[:])
    try:
        p.execute(niter=1)
        util.fail()
    except ecto.TypeMismatch, va:
        print "yeah, got typeconv error"

for E in emitters:
    typeconv(E)

sys.exit(0)
for A in acceptors:
    valuenone(A)


for E in emitters:
    for A in acceptors:
        print E, A
        do(E, A)


