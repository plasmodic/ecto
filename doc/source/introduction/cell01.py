#!/usr/bin/python 
from introduction import Printer01

print Printer01.__doc__

printer = Printer01(prefix='... ',suffix=' ...\n')

printer.inputs.message = 'TFJ'

printer.process()
