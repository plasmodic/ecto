#!/usr/bin/env python
import ecto
from ecto.opts import scheduler_options, run_plasm
from ecto import Constant
from ecto_test import Multiply, Printer
import argparse

parser = argparse.ArgumentParser(description='My awesome program thing.')

#our local command line args.
parser.add_argument('--value', metavar='VALUE', dest='value',
                    type=float, default=3, help='A value to use a constant.')
parser.add_argument('--factor', metavar='FACTOR', dest='factor',
                    type=float, default=5, help='The factor to multiply the constant with.')

#add ecto scheduler args.
group = parser.add_argument_group('ecto scheduler options')
scheduler_options(group)
options = parser.parse_args()

c = Constant(value=options.value)
m = Multiply(factor=options.factor)
pr = Printer()
plasm = ecto.Plasm()
plasm.connect(c[:] >> m[:],
              m[:] >> pr[:]
              )
#run the scheduler given the options, plasm, and our local variables.
run_plasm(options, plasm, locals=vars())

