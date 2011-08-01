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

def tpool(Scheduler):
    p = makeplasm()
    s = Scheduler(p)
    s.execute_async()
    time.sleep(0.2)
    s.stop()


for i in range(1,5):
    print "executing(Singlethreaded):",i
    tpool(Scheduler=ecto.schedulers.Singlethreaded)
    
for i in range(1,5):
    print "executing(Threadpool):",i
    tpool(Scheduler=ecto.schedulers.Threadpool)