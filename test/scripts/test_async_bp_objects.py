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

    plasm.connect(ping[:] >> sleep0[:],
                  sleep0[:] >> sleep1[:])

    return plasm

def non_async(Scheduler):
    p = makeplasm()
    s = Scheduler(p)
    s.execute(10)
    
def tpool(Scheduler):
    p = makeplasm()
    s = Scheduler(p)
    s.execute_async()
    print "execute_async"
    #this is where it fails.
    #bp::stl_input_iterator<double> begin(list_o_sleeps),end;
    time.sleep(5)
    print "time.sleep(5)"
    s.stop()
    print "s.stop()"
    s.wait()
    print "s.wait()"



print "non_async(Singlethreaded):"
non_async(Scheduler=ecto.schedulers.Singlethreaded)
print "async(Singlethreaded):"
tpool(Scheduler=ecto.schedulers.Singlethreaded)
print "non_async(Threadpool):"
non_async(Scheduler=ecto.schedulers.Threadpool)
print "async(Threadpool):"
tpool(Scheduler=ecto.schedulers.Threadpool)