#!/usr/bin/env python
import ecto
from ecto import If, TrueEveryN
from ecto_test import Generate, Printer, Add
from ecto_X import Executer

def make_executioner():
    plasm = ecto.Plasm()
    #printer = Printer("Printer", print_type='double')
    if_g = If(cell=Generate(step=1.0, start=1.0))
    truer = TrueEveryN(n=3, count=3)
    plasm.connect(truer['flag'] >> if_g['__test__'],
                  )
    executer = Executer(plasm=plasm, niters=18, outputs={'out':if_g, 'flag':truer},)
    return executer

plasm = ecto.Plasm()
executer = make_executioner()
executer2 = make_executioner()
add = Add()
printer = Printer("Printer", print_type='double')

plasm.connect(executer['out'] >> add['right'],
              executer2['out'] >> add['left'],
              add[:] >> printer[:])

ecto.view_plasm(plasm)
plasm.execute(3)


