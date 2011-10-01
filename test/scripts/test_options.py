#!/usr/bin/env python
import ecto
import ecto_test
import yaml
import argparse
from ecto.opts import scheduler_options, CellYamlFactory, cell_options, run_plasm

parser = argparse.ArgumentParser(description='My awesome program thing.')
parser.add_argument('-i,--input', metavar='IMAGE_FILE', dest='imagefile',
                    type=str, default='', help='an image file to load.')
group = parser.add_argument_group('ecto scheduler options')
scheduler_options(group, default_niter=2)


multiply_factory = cell_options(parser, ecto_test.Multiply, prefix='mult')
const_factory = cell_options(parser, ecto.Constant(value=0.50505), prefix='const')

options = parser.parse_args()
print options.mult_factor
assert options.mult_factor == 3.14
c = const_factory(options)
m = multiply_factory(options)

cyaml = CellYamlFactory(c,'const')
print cyaml.dump()
c = cyaml.load(yaml.load(cyaml.dump()))
assert c.params.value == 0.50505

pr = ecto_test.Printer()
plasm = ecto.Plasm()
plasm.connect(c[:] >> m[:],
              m[:] >> pr[:]
              )

run_plasm(options, plasm, locals=vars())

print m.outputs.out
assert m.outputs.out == 1.585857