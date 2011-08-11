#!/usr/bin/env python
import ecto
from ecto_test import Printer, Add
from ecto_X import Sink

p = Printer(print_type='double')
sink = Sink(url='localhost', port=2932)

plasm = ecto.Plasm()
plasm.connect(sink[:] >> p[:])

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Run a simple ecto client graph.')
