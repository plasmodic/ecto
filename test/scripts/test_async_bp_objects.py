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

def non_async(Scheduler):
    print "non_async"
    p = makeplasm()
    s = Scheduler(p)
    s.execute(10)
    
def tpool(Scheduler):
    print "tpool"
    p = makeplasm()
    s = Scheduler(p)
    s.execute_async()
    print "execute_async"
    #this is where it fails.
    #bp::stl_input_iterator<double> begin(list_o_sleeps),end;
    print "time.sleep(5)"
    time.sleep(5)
    print "done sleeping.  interrupting..."
    s.interrupt()
    print "done interrupting.  waiting..."
    s.wait()
    print "s.wait()"



print "async(Threadpool):"
tpool(Scheduler=ecto.schedulers.Threadpool)

print "non_async(Singlethreaded):"
non_async(Scheduler=ecto.schedulers.Singlethreaded)
print "async(Singlethreaded):"
tpool(Scheduler=ecto.schedulers.Singlethreaded)
print "non_async(Threadpool):"
non_async(Scheduler=ecto.schedulers.Threadpool)
