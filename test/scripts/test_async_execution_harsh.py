#!/usr/bin/env python
import ecto
import ecto_test
import sys, time

print "Hardware concurrency is", ecto.hardware_concurrency()

def makeplasm():
    plasm = ecto.Plasm()
    
    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.Sleep("Sleep_0", seconds=0.1)
    sleep1 = ecto_test.Sleep("Sleep_1", seconds=0.1)

    plasm.connect(ping[:] >> sleep0[:],
                  sleep0[:] >> sleep1[:])

    return plasm

p = makeplasm()
s = ecto.schedulers.Threadpool(p)
print "execute async..."
s.execute_async()
print "executing."
try:
    print "trying execute"
    s.execute()
except Exception, e:
    assert e.message == "threadpool scheduler already running"
    print e.message
