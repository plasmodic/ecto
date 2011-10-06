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

def invoke(Scheduler, whatnext):
    p = makeplasm()
    s = Scheduler(p)
    s.execute_async()
    assert s.running()
    time.sleep(0.05)
    whatnext(s)

def stoponly(s):
    s.stop()

def interruptonly(s):
    s.interrupt()

def nada(s):
    pass

def bang(Sched):
    for i in range(10):
        print "executing [%d] %s " %(i, Sched)
        invoke(Sched, nada)
        invoke(Sched, stoponly)
        invoke(Sched, interruptonly)
    
bang(ecto.schedulers.Singlethreaded)
bang(ecto.schedulers.Threadpool)

