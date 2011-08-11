#!/usr/bin/env python
import ecto
from ecto import If, TrueEveryN
from ecto_test import Generate, Printer, Add
from ecto_X import Sink, Source


plasm = ecto.Plasm()
g = Generate()

source = Source(port=2932)

plasm.connect(g['out'] >> source['in'],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute_async()

def scope():
    p = Printer(print_type='double')
    sink = Sink(url='localhost', port=2932)
    
    plasm2 = ecto.Plasm()
    plasm2.connect(sink[:] >> p[:])
    sched2 = ecto.schedulers.Singlethreaded(plasm2)
    sched2.execute(10)

scope()
