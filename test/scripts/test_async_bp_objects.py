#!/usr/bin/env python
import ecto
import ecto_test
import sys, time
import random

def makeplasm():
    plasm = ecto.Plasm()
    
    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.SleepPyObjectAbuser(list_o_sleeps=[random.random() * 0.1 for i in range(1,10)])
    sleep1 = ecto_test.SleepPyObjectAbuser(list_o_sleeps=[random.random() * 0.1 for i in range(1,10)])
    sleep2 = ecto_test.SleepPyObjectAbuser(list_o_sleeps=[random.random() * 0.1 for i in range(1,10)])
    sleep3 = ecto_test.SleepPyObjectAbuser(list_o_sleeps=[random.random() * 0.1 for i in range(1,10)])

    plasm.connect(ping[:] >> sleep0[:],
                  sleep0[:] >> sleep1[:],
                  sleep1[:] >> sleep2[:],
                  sleep2[:] >> sleep3[:])

    return plasm

def async(s):
    print "s.execute_async"
    s.execute_async(niter=5)

def sync(s):
    print "s.execute"
    s.execute(niter=5)
    print "s.execute DONE"
    
def nada(s):
    print "nada"

def waitonly(s):
    print "waitonly"
    s.wait()


def interrupt(s):
    print "interrupt"
    s.interrupt()

def interrupt_and_wait(s):
    print "interrupt_and_wait"
    s.interrupt()
    s.wait()

def tpool(Scheduler, go, afterwards, sleepdur=0.1):
    print "tpool"
    p = makeplasm()
    s = Scheduler(p)
    go(s)
    #this is where it fails.
    #bp::stl_input_iterator<double> begin(list_o_sleeps),end;
    print "time.sleep(", sleepdur, ")"
    stime = time.time()
    time.sleep(sleepdur)
    afterwards(s)
    etime = time.time()
    print "elapsed", etime-stime

def doemall(Sched):
    tpool(Sched, async, waitonly)
    tpool(Sched, async, interrupt_and_wait)
    tpool(Sched, async, interrupt)
    tpool(Sched, async, nada)
    tpool(Sched, sync, nada)
    tpool(Sched, sync, interrupt)
    tpool(Sched, sync, interrupt_and_wait)
    tpool(Sched, sync, waitonly)
    
doemall(ecto.schedulers.Singlethreaded)
doemall(ecto.schedulers.Threadpool)

