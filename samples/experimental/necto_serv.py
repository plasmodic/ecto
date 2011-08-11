#!/usr/bin/env python
import ecto
from ecto_test import Generate
from ecto_X import Source

plasm = ecto.Plasm()
g = Generate()

source = Source(port=2932)

plasm.connect(g['out'] >> source['in'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Run a simple ecto server graph.')
